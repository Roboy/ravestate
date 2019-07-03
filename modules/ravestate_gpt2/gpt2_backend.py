import torch
import torch.nn.functional as F
from pytorch_pretrained_bert import GPT2Tokenizer, GPT2LMHeadModel

from reggol import get_logger
logger = get_logger(__name__)

# TODO parameter finetuning
TEMPERATURE = 0.5
LENGTH = 30
TOP_K = 40
BATCH_SIZE = 1


def top_k_logits(logits, k):
    """
    Masks everything but the k top entries as -infinity (1e10).
    Used to mask logits such that e^-infinity -> 0 won't contribute to the
    sum of the denominator.
    """
    if k == 0:
        return logits
    else:
        values = torch.topk(logits, k)[0]
        batch_mins = values[:, -1].view(-1, 1).expand_as(logits)
        return torch.where(logits < batch_mins, torch.ones_like(logits) * -1e10, logits)


class GPT2Responder:
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    tokenizer = GPT2Tokenizer.from_pretrained('gpt2-medium')
    model = GPT2LMHeadModel.from_pretrained('gpt2-medium')
    model.to(device)
    model.eval()

    def process(self, prompt: str):
        tokens = self.tokenizer.encode(prompt)
        tokens_tensor = torch.tensor(tokens, device=self.device, dtype=torch.long).unsqueeze(0).repeat(BATCH_SIZE, 1)

        prev = tokens_tensor
        output = tokens_tensor
        past = None
        with torch.no_grad():
            for i in range(LENGTH):
                hidden_states, past = self.model(prev, past=past)
                hidden_states = hidden_states[:, -1, :] / TEMPERATURE
                hidden_states = top_k_logits(hidden_states, k=TOP_K)
                log_probs = F.softmax(hidden_states, dim=-1)
                prev = torch.multinomial(log_probs, num_samples=1)
                output = torch.cat((output, prev), dim=1)

        output = output[:, len(tokens):].tolist()
        response = self.tokenizer.decode(output[0])
        return response
