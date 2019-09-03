import re
from typing import Dict
from roboy_parlai import wildtalk

from reggol import get_logger
logger = get_logger(__name__)

fix_spaces = re.compile(r'\s*([?!.,]+(?:\s+[?!.,]+)*)\s*')


class Parlai_Responder:
    def process(self, prompt: str, model_options: Dict[str, str]):
        result = wildtalk(prompt)
        return fix_spaces.sub(lambda x: "{} ".format(x.group(1).replace(" ", "")), result)
