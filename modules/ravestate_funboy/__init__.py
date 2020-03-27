import random
import requests
from statistics import mean, mode

import ravestate as rs
import ravestate_nlp as nlp
import ravestate_verbaliser as verbaliser
import ravestate_idle as idle
import ravestate_rawio as rawio
import ravestate_ontology
import ravestate_interloc as interloc
import ravestate_nlp as nlp

from os.path import realpath, dirname, join
from datetime import datetime

from scientio.ontology.ontology import Ontology
from scientio.session import Session
from scientio.ontology.node import Node

from .comedian_strategy import ComedianStrategy
from .emotion_strategy import EmotionStrategy
from .wildtalk_server import run as server

from reggol import get_logger
logger = get_logger(__name__)


MODEL_KEY = "model"
SERVER_ADDRESS = "server_address"
SERVER_PORT = "server_port"
TEMPERATURE = "temperature"
MAX_LENGTH = "max_length"
TOP_K = "top_k"
TOP_P = "top_p"
MAX_HISTORY = "max_history"

CONFIG = {
    SERVER_ADDRESS: "http://127.0.0.1",  # can be changed if server is running on its own on a separate machine
    SERVER_PORT: 5100,
    TEMPERATURE: 0.9,  # convai_gpt, gpt2: higher value -> more variation in output
    MAX_LENGTH: 20,  # convai_gpt, gpt2: maximal length of generated output
    TOP_K: 0,  # convai_gpt, gpt2: <=0: no filtering, >0: keep only top k tokens with highest probability.
    TOP_P: 0.9,  # convai_gpt: <=0.0 no filtering, >0.0: keep smallest subset whose total probability mass >= top_p
    MAX_HISTORY: 4,  # convai_gpt: maximal number of previous dialog turns to be used for output generation
}

server_started = False

verbaliser.add_file(join(dirname(realpath(__file__)), "jokes_data.yml"))


with rs.Module(
        name="funboy",
        config=CONFIG,
        depends=(verbaliser.mod, rawio.mod, interloc.mod, nlp.mod)) as mod:

    sig_tell_joke = rs.Signal(name="tell_joke")

    sig_joke_told = rs.Signal(name="joke_told")
    sig_joke_finish = rs.Signal(name="joke_finish")

    prop_start_timestamp = rs.Property(name="start_timestamp")
    prop_joke_category = rs.Property(name="joke_category")
    prop_comedian = rs.Property(name="comedian")
    prop_emotion = rs.Property(name="emotion")
    prop_scores = rs.Property(name="scores")
    prop_input = rs.Property(name="input")
    prop_do_joke = rs.Property(name="do_joke")
    prop_server = rs.Property(name="address")
    prop_validation = rs.Property(name="validation_score")


    @rs.state(cond=rs.sig_startup,
              write=(prop_start_timestamp, prop_comedian, prop_emotion, prop_scores, prop_server, prop_validation),
              read=prop_emotion)
    def start(ctx):
        logger.info("Starting funboy")
        ctx[prop_scores] = {
            "chicken": 0,
            "momma": 0,
            "dadjokes": 0,
            "friend": 0,
            "religion": 0,
            "pet": 0,
            "bar": 0,
            "clown": 0,
            "trump": 0,
            "german": 0,
            "army": 0,
            "other": 0,
            "boss": 0,
            "doctor": 0,
            "british": 0,
            "police": 0,
            "cookie": 0,
            "family": 0
        }
        ctx[prop_start_timestamp] = datetime.now()
        server_address = f"{ctx.conf(key=SERVER_ADDRESS)}:{ctx.conf(key=SERVER_PORT)}"
        if not wildtalk_server_up(server_address):
            global server_started
            server_started = True
            logger.info("Starting up wildtalk server")
            server(port=ctx.conf(key=SERVER_PORT))  # start server
            server_started = False
        ctx[prop_server] = server_address

        ctx[prop_validation] = 0
        ctx[prop_comedian] = ComedianStrategy()
        ctx[prop_emotion] = EmotionStrategy(alpha=1, beta=0)
        ctx[prop_emotion].configure(video=True, audio=False)

    @rs.state(cond=nlp.prop_triples.changed(),
              read=(nlp.prop_triples, rawio.prop_in, prop_server, prop_start_timestamp),
              write=(prop_input, prop_do_joke),
              signal=sig_tell_joke)
    def decide(ctx):
        logger.info("Deciding funboy")
        triple = ctx[nlp.prop_triples][0]
        prompt = ctx[rawio.prop_in]
        ctx[prop_input] = prompt

        do_jokes = False

        if not wildtalk_server_up(ctx[prop_server]):
            do_jokes = True
        else:
            if "joke" in prompt:
                do_jokes = True
            else:
                if (datetime.now() - ctx[prop_start_timestamp]).seconds > 60:
                    if triple.is_question:
                        do_jokes = False if random.random() < 0.5 else True
                    else:
                        do_jokes = True
                else:
                    do_jokes = False

        ctx[prop_do_joke] = do_jokes

        return rs.Emit()

    @rs.state(cond=sig_tell_joke,
              write=(prop_joke_category, prop_scores),
              read=(interloc.prop_all, prop_input, prop_do_joke, prop_scores, prop_validation))
    def assess(ctx):
        logger.info("Assessing funboy")
        session: Session = ravestate_ontology.get_session()
        ontology: Ontology = ravestate_ontology.get_ontology()
        prompt = ctx[prop_input]
        do_jokes = ctx[prop_do_joke]

        category = None

        if do_jokes:
            logger.info("Assessing funboy")

            category = define_type(prompt)

            if category is None:
                node = None
                # Only 1 interlocutor
                for i in ctx.enum(interloc.prop_all):
                    node = ctx[i]
                # interact with the Node via scientio
                # print(node)
                if node is not None:
                    # Get associated joketypes
                    # Retrieve the affinity values
                    jokenode = Node()
                    jokenode.set_type(ontology.get_type(entity="get_type"))
                    jokenodes = session.retrieve(jokenode)
                    interloc_id = node.get_id()
                    affinities = {x.get_name(): x.get_relationships("AFFINITY")[interloc_id] for x in jokenodes}
                    ctx[prop_scores] = affinities

                scores = ctx[prop_scores]
                mu = mean(ctx[prop_scores].values())
                above_mean = [x for x, y in scores.items() if y >= mu]
                below_mean = [x for x, y in scores.items() if y < mu]

                threshold = int(len(scores) * 2 / 3)
                if len(above_mean) > threshold:
                    above_mean = above_mean[-threshold:]
                    below_mean.extend(above_mean[:-threshold])
                elif len(below_mean) > threshold:
                    below_mean = below_mean[:threshold]
                    above_mean.extend(below_mean[threshold:])

                if ctx[prop_validation] >= 0:
                    category = random.choice(above_mean)
                else:
                    category = random.choice(below_mean)
        else:
            category = None

        logger.info(f"Selected type: {category}")
        ctx[prop_joke_category] = category
        return rs.Emit()


    @rs.state(cond=sig_tell_joke & prop_joke_category.changed(),
              read=(prop_comedian, prop_input, prop_do_joke, prop_joke_category, prop_server),
              write=rawio.prop_out,
              signal=sig_joke_told)
    def render(ctx):
        """
        Do GPT magic
        :param ctx:
        :return:
        """
        logger.info("Rendering funboy")
        prompt = ctx[prop_input]
        do_jokes = ctx[prop_do_joke]
        category = ctx[prop_joke_category]
        address = ctx[prop_server]

        if do_jokes and category is not None:
            logger.info("Rendering funboy")
            utterance = None if random.random() < 0.7 else prompt
            joke = ctx[prop_comedian].render(type=ctx[prop_joke_category], utterance=utterance)

            if random.random() > 0.7:
                joke = f"{verbaliser.get_random_phrase('segue')} {joke}"
            if random.random() > 0.5:
                joke = f"{joke}. {verbaliser.get_random_phrase('haha')}"

            ctx[rawio.prop_out] = joke
        else:
            """
            Wildtalk using a HTTP server.
            Post input and get answer through the HTTP interface.
            """
            params = {'prompt': prompt}
            for key in [TEMPERATURE, MAX_LENGTH, TOP_K, TOP_P, MAX_HISTORY]:
                params[key] = ctx.conf(key=key)
            sample = None
            while not sample:  # sample might be "fully sanitized" and empty
                response = requests.get(address, params=params)
                response_json = response.json()
                logger.info(response_json)
                sample = response_json['response'].strip()
            ctx[rawio.prop_out] = sample

        return rs.Emit()

    @rs.state(cond=sig_joke_told,
              write=prop_validation,
              read=prop_emotion)
    def validate(ctx):
        positivity_scores = ctx[prop_emotion].get_positivity()
        ctx[prop_validation] = mode(positivity_scores)

    @rs.state(cond=sig_joke_told & prop_validation.changed(),
              read=(prop_joke_category, prop_validation, interloc.prop_all, prop_scores),
              write=prop_scores,
              signal=sig_joke_finish)
    def attune(ctx):
        logger.info("Attuning funboy")
        session: Session = ravestate_ontology.get_session()
        ontology: Ontology = ravestate_ontology.get_ontology()
        val_score = ctx[prop_validation]

        if ctx[prop_joke_category] is not None:
            affinity = ctx[prop_scores][ctx[prop_joke_category]]

            if val_score == 1:
                affinity += 1
            elif val_score == -1:
                affinity = max(0, affinity - 1)

            # Reassign the affinity to the JokeType
            ctx[prop_scores][ctx[prop_joke_category]] = affinity

            interloc_node = None
            # Only 1 interlocutor
            for i in ctx.enum(interloc.prop_all):
                interloc_node = ctx[i]
            # interact with the Node via scientio
            # print(node)
            if interloc_node is not None:
                # Get associated joketypes
                # Set the affinity values
                jokenode = Node()
                jokenode.set_type(ontology.get_type(entity="get_type"))
                jokenode.set_name(ctx[prop_joke_category])
                jokenode = session.retrieve(jokenode)[0]
                interloc_node.set_relationships({"AFFINITY": (jokenode.get_id(), affinity)})

            logger.info(f"Scores: {ctx[prop_scores]}")

        return rs.Emit()


def wildtalk_server_up(server_address):
    try:
        status = requests.head(server_address).status_code
    except requests.exceptions.RequestException or requests.exceptions.ConnectionError:
        status = None
    return status == 200


def define_type(s: str):
    _s = s.lower()
    _sl = s.split("\t\n\r\f,.;:!?'\"()")

    if "chicken" in _s:
        return f"chicken"
    elif "yo momma" in _s or \
            "your momma" in _s:
        return f"momma"
    elif "trump" in _sl:
        return f"trump"
    elif "cat" in _sl or \
            "dog" in _sl or \
            "pet" in _sl:
        return f"pet"
    elif "army" in _sl or \
            "militar" in _sl:
        return f"army"
    elif "police" in _sl:
        return f"police"
    elif "catholic" in _s or \
            "jesus" in _s or \
            "god" in _s or \
            "buddh" in _s or \
            "islam" in _s or \
            "muslim" in _s:
        return f"religion"
    elif "bar" in _sl or \
            "barman" in _s or \
            "bartender" in _s:
        return f"bar"
    elif "clown" in _s \
            or "circus" in _s:
        return f"clown"
    elif "german" in _s:
        return f"german"
    elif "boss" in _sl:
        return f"boss"
    elif "doctor" in _s:
        return f"doctor"
    elif "english" in _s or \
            "british" in _s:
        return f"british"
    elif "cookie" in _s or \
            "biscuit" in _s or \
            "shortbread" in _s:
        return f"cookie"
    elif "mom" in _sl or \
            "mum" in _sl or \
            "momma" in _sl or \
            "mother" in _s or \
            "brother" in _s or \
            "sister" in _s or \
            "father" in _s or \
            "grandfather" in _s or \
            "grandpa" in _s or \
            "grandmother" in _s or \
            "grandma" in _s or \
            "granny" in _s:
        return f"family"
    elif "friend" in _s:
        return f"friend"

    return None
