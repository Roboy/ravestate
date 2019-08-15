```
 _    _  _____  _     ______  _____   ___   _      _   __
| |  | ||_   _|| |    |  _  \|_   _| / _ \ | |    | | / /
| |  | |  | |  | |    | | | |  | |  / /_\ \| |    | |/ / 
| |/\| |  | |  | |    | | | |  | |  |  _  || |    |    \ 
\  /\  / _| |_ | |____| |/ /   | |  | | | || |____| |\  \
 \/  \/  \___/ \_____/|___/    \_/  \_| |_/\_____/\_| \_/
```

## WILDTALK

The wildtalk module module generates responses for given input using the given model. 

In order to change the model used for generation, change this option:

| Option | Description |
|---|---|
| model | one of "convai_gpt", "gpt2", "parlai" |

If no separate wildtalk server is running, this module starts a wildtalk server which can be used by multiple ravestate instances.

### Available Models
#### ConvAI GPT
This model is based on [transfer-learning-conv-ai](https://github.com/huggingface/transfer-learning-conv-ai)

It can be used by setting model to "convai_gpt"

| Option | Description |
|---|---|
| temperature | higher value -> more variation in output |
| max_length | maximal length of generated output |
| top_k | <=0: no filtering, >0: keep only top k tokens with highest probability. |
| top_p | <=0.0 no filtering, >0.0: keep smallest subset whose total probability mass >= top_p |
| max_history | maximal number of previous dialog turns to be used for output generation |

#### GPT2
This model uses the gpt2 transformer and model from 
[pytorch_transformers](https://github.com/huggingface/pytorch-transformers).
It uses the gpt2-medium dataset.

It can be used by setting model to "gpt2"

| Option | Description |
|---|---|
| temperature | higher value -> more variation in output |
| max_length | maximal length of generated output |
| top_k | <=0: no filtering, >0: keep only top k tokens with highest probability. |

#### Parlai
This model uses [roboy-parlai](https://pypi.org/project/roboy-parlai/).

It can be used by setting model to "parlai"

| Option | Description |
|---|---|
| temperature | higher value -> more variation in output |
| max_length | maximal length of generated output |
| top_k | <=0: no filtering, >0: keep only top k tokens with highest probability. |

### Separate Server
A separate server can be used for running the wildtalk generation.
This can be configured with the following options:

| Option | Description |
|---|---|
| server_address | Address under which the server is accessible |
| server_port | Port of the server |

To start a standalone server for wildtalk generation, execute this in the /modules folder of ravestate:

`python -c "from ravestate_wildtalk import server; server.run(port=<PORTNUMBER>, model=<MODELNAME>)`

Note that the model used on server startup will be the model used when accessed from ravestate. 
