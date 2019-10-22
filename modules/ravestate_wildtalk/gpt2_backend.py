import re
from typing import Dict

import torch
import torch.nn.functional as F
from pytorch_transformers import GPT2Tokenizer, GPT2LMHeadModel

from reggol import get_logger
logger = get_logger(__name__)


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


class GPT2_Responder:
    def __init__(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # TODO maybe smaller gpt2 model separately
        self.tokenizer = GPT2Tokenizer.from_pretrained('gpt2-medium')
        self.model = GPT2LMHeadModel.from_pretrained('gpt2-medium')
        self.model.to(self.device)
        self.model.eval()

    def process(self, prompt: str, model_options: Dict[str, str]):
        temperature = float(model_options["temperature"])
        max_length = int(model_options["max_length"])
        top_k = float(model_options["top_k"])

        tokens = self.tokenizer.encode(f"You: {prompt} Roboy: ")
        tokens_tensor = torch.tensor(tokens, device=self.device, dtype=torch.long).unsqueeze(0)

        prev = tokens_tensor
        output = tokens_tensor
        past = None
        with torch.no_grad():
            for i in range(max_length):
                hidden_states, past = self.model(prev, past=past)
                hidden_states = hidden_states[:, -1, :] / temperature
                hidden_states = top_k_logits(hidden_states, k=top_k)
                log_probs = F.softmax(hidden_states, dim=-1)
                prev = torch.multinomial(log_probs, num_samples=1)
                output = torch.cat((output, prev), dim=1)

        output = output[:, len(tokens):].tolist()
        response = self.tokenizer.decode(output[0])
        return sanitize_response(response)


def sanitize_response(response: str):
    # remove groups of non-letters (incl. surrounding whitespace) e.g. ??????
    response = re.sub(r'\s*[^\w\s]{3,}\s*', '', response)

    # find first occurrence of <NAME>:
    speaker_prompt = re.search(r'\S+:', response)
    if speaker_prompt:
        # only use utterance before speaker prompt
        response = response[:speaker_prompt.start()]
    else:
        # cut off from the right until one of .?! is found to avoid half-sentences
        last_period = response.rfind('.')
        last_qmark = response.rfind('?')
        last_exmark = response.rfind('!')
        if last_period > 0:
            response = response[:last_period + 1]
        elif last_exmark > 0:
            response = response[:last_exmark + 1]
        elif last_qmark > 0:
            response = response[:last_qmark + 1]
    return response.strip()
