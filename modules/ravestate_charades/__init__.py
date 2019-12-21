import ravestate as rs
import ravestate_rawio as rawio
import ravestate_nlp as nlp
import ravestate_idle as idle
import ravestate_verbaliser as verbaliser

import random
from os.path import realpath, dirname, join
from reggol import get_logger
logger = get_logger(__name__)

verbaliser.add_folder(join(dirname(realpath(__file__)), "charades_phrases"))

ROS_AVAILABLE = False
try:
    import rospy
    from ha_recognition.srv import RecordActivity
    from ha_recognition.msg import ActivityLabel
    from std_msgs.msg import String
    import ravestate_ros1 as ros1
    ROS_AVAILABLE = True
except ImportError as e:
    logger.error(f"""
--------
An exception occured during imports: {e}
Please make sure to have the following items installed & sourced:
1. ROS melodic
2. roboy_communication
--------
    """)

BROKEN_MESSAGE = "Sorry, something went wrong. Let's try again later"

# emotions
SHY_EMOTION = "shy"
HAPPY_EMOTION = "happy"
LUCKY_EMOTION = "lucky"
KISS_EMOTION = "kiss"
ROLL_EYES_EMOTION = "rolling_eyes"
SUNGLASSES_ON_EMOTION = "sunglasses_on"
LOOK_LEFT_EMOTION = "lookleft"
LOOK_RIGHTEMOTION = "lookright"

if ROS_AVAILABLE:

    with rs.Module(name="charades",
                   depends=(nlp.mod,
                            idle.mod,
                            rawio.mod,
                            ros1.mod,
                            verbaliser.mod)) as mod:

        # signals
        sig_mentioned_charades = rs.Signal("mentioned_charades")
        sig_offered_charades = rs.Signal("offer_game")
        sig_play_decision_processed = rs.Signal("play_decision_processed")
        sig_rules_decision_done = rs.Signal("rules_decision_done")
        sig_activity_choice_prompted = rs.Signal("activity_choice_propmted")
        sig_readiness_check_started = rs.Signal("readiness_check_started")
        sig_action_captured = rs.Signal("action_captured")
        sig_label_named = rs.Signal("action_captured")
        sig_asked_to_continue = rs.Signal("asked_to_continue")
        sig_continue_game = rs.Signal("continue_game")
        sig_feedback_unclear = rs.Signal("feedback_not_understood")
        sig_feedback_repeat = rs.Signal("feedback_repeat")

        # properties
        prop_starting_state = rs.Property(name="prop_starting_state",
                                          always_signal_changed=False,
                                          default_value=True)
        prop_stop_game = rs.Property(name="prop_stop_game",
                                     always_signal_changed=True,
                                     default_value=False)
        prop_choice_unclear = rs.Property(name="prop_choice_unclear",
                                          always_signal_changed=True,
                                          default_value="")
        prop_activity_chosen = rs.Property(name="prop_activity_chosen",
                                           always_signal_changed=True,
                                           default_value="")
        prop_game_in_progress = rs.Property(name="prop_game_in_progress",
                                            always_signal_changed=True,
                                            default_value=False)
        prop_waiting_for_label = rs.Property(name="prop_waiting_for_label",
                                             always_signal_changed=True,
                                             default_value=False)
        prop_feedback_received = rs.Property(name="prop_feedback_received",
                                             always_signal_changed=True,
                                             default_value=False)
        prop_ping_choice_count = rs.Property(name="prop_ping_choice_count",
                                             always_signal_changed=True,
                                             default_value=0)
        prop_continuation_unclear = rs.Property(
            name="prop_continuation_unclear",
            always_signal_changed=True,
            default_value=False)
        prop_guess_attempt_count = rs.Property(name="prop_guess_attempt_count",
                                               always_signal_changed=True,
                                               default_value=0)
        prop_another_attempt = rs.Property(name="prop_another_attempt",
                                           always_signal_changed=True,
                                           default_value=False)

        # ros properties

        # not used for now. use direct client call with recognition_client()
        # prop_record_action = ros1.Ros1CallProperty(name="_recognition",
        #                                            service_name="ha_recognition",
        #                                            service_type=RecordActivity,
        #                                            call_timeout=5.0)
        prop_label_subscriber = ros1.Ros1SubProperty(
            name="label_subsriber",
            topic="/ha_classifier",
            msg_type=ActivityLabel,
            always_signal_changed=True)

        prop_emotion_publisher = ros1.Ros1PubProperty(
            name="emotion_publisher",
            topic="/roboy/cognition/face/show_emotion",
            msg_type=String)

        def recognition_client():
            rospy.wait_for_service("ha_recognition")
            try:
                record = rospy.ServiceProxy("ha_recognition", RecordActivity)
                resp = record()
                if resp:
                    return True
            except rospy.ServiceException as e:
                print('Service call failed: ', e)

        # general states

        @rs.state(cond=nlp.prop_tokens.changed(),
                  read=(nlp.prop_lemmas, prop_game_in_progress),
                  signal=sig_mentioned_charades)
        def signal_charades(ctx: rs.ContextWrapper):
            '''
            reacts to mentioning charades with a sig_mentioned_charades signal
            '''
            if ('charade' in ctx[nlp.prop_lemmas] and
                    not ctx[prop_game_in_progress] and
                    'stop' not in ctx[nlp.prop_lemmas]):
                return rs.Emit()
            else:
                return rs.Resign()

        # @rs.state(cond=(nlp.prop_tokens.changed() &
        #                 sig_offered_charades.max_age(-1)),
        #           read=nlp.prop_lemmas,
        #           write=prop_stop_game)
        # def signal_stop_the_game(ctx: rs.ContextWrapper):
        #     if ('stop' in ctx[nlp.prop_lemmas] and
        #             'charade' in ctx[nlp.prop_lemmas]):
        #         ctx[prop_stop_game] = True
        #     else:
        #         return rs.Resign()

        @rs.state(cond=(idle.sig_bored |
                        prop_stop_game.changed()),
                  read=prop_starting_state,
                  write=(rawio.prop_out,
                         prop_game_in_progress,
                         prop_waiting_for_label,
                         prop_starting_state,
                         prop_ping_choice_count))
        def stop_game_session(ctx: rs.ContextWrapper):
            '''
            go off the game session so new game can be started
            '''
            if not ctx[prop_starting_state]:
                ctx[prop_game_in_progress] = False
                ctx[prop_waiting_for_label] = False
                ctx[prop_starting_state] = True
                ctx[prop_ping_choice_count] = 0
                ctx[rawio.prop_out] = "Stopping game session"

        # main game convo flow

        @rs.state(cond=(nlp.sig_intent_play |
                        sig_mentioned_charades |
                        (sig_play_decision_processed &
                         prop_choice_unclear.changed())),
                  read=(nlp.prop_tokens, prop_game_in_progress),
                  write=(rawio.prop_out, prop_starting_state),
                  signal=sig_offered_charades,
                  emit_detached=True)
        def offer_game(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_offer_game")
                ctx[prop_starting_state] = False
                return rs.Emit()
            else:
                rs.Resign()

        @rs.state(cond=(sig_offered_charades.max_age(10) &
                        nlp.prop_tokens.changed()),
                  read=(nlp.prop_yesno, prop_game_in_progress, prop_stop_game),
                  write=(rawio.prop_out,
                         prop_choice_unclear,
                         prop_stop_game,
                         prop_emotion_publisher),
                  signal=sig_play_decision_processed,
                  emit_detached=True)
        def process_play_decision(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                if ctx[nlp.prop_yesno].yes():
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                        "charades_positive_expressions") + \
                        " Do you want to hear the rules?"
                elif ctx[nlp.prop_yesno].no():
                    if random.random() < 0.5:
                        ctx[prop_emotion_publisher] = String(
                            data=ROLL_EYES_EMOTION)
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                        "charades_refuse_offer")
                    ctx[prop_stop_game] = True
                else:
                    ctx[prop_choice_unclear] = True
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                        "charades_misunderstanding")
                return rs.Emit()
            else:
                return rs. Resign()

        @rs.state(cond=(nlp.prop_yesno.changed() &
                        sig_play_decision_processed.max_age(10)),
                  read=(nlp.prop_yesno, prop_game_in_progress),
                  write=(rawio.prop_out, prop_game_in_progress),
                  signal=sig_rules_decision_done,
                  emit_detached=True)
        def react_to_rules_decision(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                rules = "Ok. So you pick an activity and demonstrate it to " \
                    "me and I try to guess what it is that you are showing. " \
                    "I know a lot of activities, but not all in the world. " \
                    "You can consult my tablet to pick one of the actions. " \
                    "I have three attempts at guessing the right activity." \
                    "When I will actually start recording I will " \
                    "say 'Beep' so you can start the demonstration of the " \
                    "activity. You will have about three seconds to show " \
                    "the activity to me until I say beep again."
                if ctx[nlp.prop_yesno].no():
                    out = "Ok, let's get to the game then"
                elif ctx[nlp.prop_yesno].yes():
                    out = rules
                else:
                    out = "I will count that as no."
                ctx[rawio.prop_out] = out
                ctx[prop_game_in_progress] = True
                return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=sig_rules_decision_done | sig_continue_game,
                  write=rawio.prop_out,
                  signal=sig_activity_choice_prompted,
                  emit_detached=True)
        def offer_activity_choice(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                "charades_offer_activity_choice")
            return rs.Emit(wipe=True)

        @rs.state(cond=(nlp.prop_tokens.changed() &
                        sig_activity_choice_prompted.max_age(15)),
                  read=(nlp.prop_lemmas, prop_game_in_progress),
                  write=(rawio.prop_out,
                         prop_choice_unclear,
                         prop_activity_chosen,
                         prop_ping_choice_count),
                  signal=sig_readiness_check_started, emit_detached=True)
        def react_to_choice(ctx: rs.ContextWrapper):
            if ctx[prop_game_in_progress]:
                if "start" in ctx[nlp.prop_lemmas]:
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                        "charades_countdown")
                    ctx[prop_activity_chosen] = True
                    ctx[prop_ping_choice_count] = 0
                else:
                    ctx[prop_choice_unclear] = True
                return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=(sig_activity_choice_prompted.min_age(16).max_age(-1)),
                  read=prop_ping_choice_count,
                  write=(rawio.prop_out,
                         prop_ping_choice_count,
                         prop_stop_game,
                         prop_emotion_publisher),
                  signal=sig_activity_choice_prompted,
                  emit_detached=True)
        def ping_activity_choice(ctx: rs.ContextWrapper):
            rand = random.random()
            pinged_times = ctx[prop_ping_choice_count]
            if pinged_times < 5:
                if rand < 0.15:
                    ctx[prop_emotion_publisher] = String(
                        data=LOOK_RIGHTEMOTION)
                elif rand > 0.15 and rand < 0.3:
                    ctx[prop_emotion_publisher] = String(
                        data=LOOK_LEFT_EMOTION)
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_ping_activity_choice")
                ctx[prop_ping_choice_count] = pinged_times + 1
                return rs.Emit()
            else:
                ctx[prop_stop_game] = True

        @rs.state(cond=(sig_readiness_check_started &
                        prop_choice_unclear.changed()),
                  write=rawio.prop_out,
                  signal=sig_activity_choice_prompted,
                  emit_detached=True)
        def react_to_unclear_choice(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                "charades_misunderstanding") + " " + \
                "Say 'start' when you're ready"
            return rs.Emit()

        @rs.state(cond=(sig_readiness_check_started &
                        prop_activity_chosen.changed()),
                  write=(rawio.prop_out, prop_waiting_for_label,
                         prop_guess_attempt_count),
                  signal=sig_action_captured,
                  emit_detached=True)
        def start_recording(ctx: rs.ContextWrapper):
            resp = recognition_client()
            if resp:
                ctx[rawio.prop_out] = "Beep! Now let me think a little bit"
                ctx[prop_waiting_for_label] = True
                ctx[prop_guess_attempt_count] = 0
                return rs.Emit()
            else:
                ctx[rawio.prop_out] = BROKEN_MESSAGE

        @rs.state(cond=(prop_label_subscriber.changed() |
                        prop_another_attempt.changed()),
                  read=(prop_label_subscriber, prop_waiting_for_label,
                        prop_guess_attempt_count),
                  write=(rawio.prop_out, prop_waiting_for_label,
                         prop_feedback_received, prop_guess_attempt_count),
                  signal=sig_label_named,
                  emit_detached=True)
        def name_the_label(ctx: rs.ContextWrapper):
            if ctx[prop_waiting_for_label] or (ctx[prop_guess_attempt_count] > 0 and
                                               ctx[prop_guess_attempt_count] < 3):
                message = ctx[prop_label_subscriber]
                if message:
                    confidence = message.confidence[ctx[prop_guess_attempt_count]]
                    label = message.label[ctx[prop_guess_attempt_count]]
                    if confidence < 0.5:
                        conf_expr = verbaliser.get_random_phrase(
                            "charades_lower_confidence_prediction")
                    elif confidence < 0.9:
                        conf_expr = verbaliser.get_random_phrase(
                            "charades_higher_confidence_prediction")
                    else:
                        conf_expr = verbaliser.get_random_phrase(
                            "charades_highest_confidence_prediction")
                    ctx[rawio.prop_out] = conf_expr + " " + str(label) +\
                        " " + verbaliser.get_random_phrase(
                        "charades_ask_for_feedback")
                    ctx[prop_waiting_for_label] = False
                    ctx[prop_feedback_received] = False
                    ctx[prop_guess_attempt_count] = ctx[prop_guess_attempt_count] + 1
                    return rs.Emit()
                else:
                    ctx[rawio.prop_out] = BROKEN_MESSAGE
            else:
                return rs.Resign()

        @rs.state(cond=((sig_label_named.max_age(10) &
                         nlp.prop_yesno.changed()) |
                        (sig_feedback_repeat.max_age(10) &
                         nlp.prop_yesno.changed())),
                  read=(nlp.prop_yesno, prop_game_in_progress,
                        prop_guess_attempt_count),
                  write=(rawio.prop_out,
                         prop_feedback_received,
                         prop_emotion_publisher,
                         prop_another_attempt),
                  signal=sig_feedback_unclear,
                  emit_detached=True)
        def react_to_feedback(ctx: rs.ContextWrapper):
            if ctx[prop_game_in_progress]:
                rand = random.random()
                if ctx[nlp.prop_yesno].yes():
                    ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                        "charades_winning_exclamations")
                    if rand < 0.5:
                        emotion = HAPPY_EMOTION
                    else:
                        emotion = SUNGLASSES_ON_EMOTION
                    ctx[prop_emotion_publisher] = String(data=emotion)
                    ctx[prop_feedback_received] = True
                elif ctx[nlp.prop_yesno].no():
                    if ctx[prop_guess_attempt_count] < 3:
                        ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                            "charades_new_guess_attempt")
                        ctx[prop_another_attempt] = True
                    else:
                        ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                            "charades_losing_exclamations")
                        if rand < 0.8:
                            ctx[prop_emotion_publisher] = String(data=SHY_EMOTION)
                        ctx[prop_feedback_received] = True
                else:
                    ctx[prop_feedback_received] = False
                    return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=sig_feedback_unclear,
                  read=(prop_feedback_received, prop_game_in_progress),
                  write=rawio.prop_out,
                  signal=sig_feedback_repeat,
                  emit_detached=True)
        def feedback_not_clear(ctx: rs.ContextWrapper):
            if ctx[prop_game_in_progress] and not ctx[prop_feedback_received]:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_misunderstanding") + " " + \
                    verbaliser.get_random_phrase("charades_ask_for_feedback")
                return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=sig_action_captured.min_age(20).max_age(-1),
                  read=(prop_waiting_for_label, prop_feedback_received),
                  write=rawio.prop_out)
        def no_prediction(ctx: rs.ContextWrapper):
            if ctx[prop_waiting_for_label] and not ctx[prop_feedback_received]:
                ctx[rawio.prop_out] = BROKEN_MESSAGE

        @rs.state(cond=(prop_feedback_received.changed() |
                        prop_continuation_unclear.changed()),
                  read=prop_feedback_received,
                  write=rawio.prop_out,
                  signal=sig_asked_to_continue,
                  emit_detached=True)
        def ask_to_continue(ctx: rs.ContextWrapper):
            if ctx[prop_feedback_received]:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_offer_another_round")
                return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=(nlp.prop_yesno.changed() &
                        sig_asked_to_continue.max_age(10)),
                  read=nlp.prop_yesno,
                  write=(rawio.prop_out,
                         prop_game_in_progress,
                         prop_continuation_unclear,
                         prop_stop_game,
                         prop_emotion_publisher),
                  signal=sig_continue_game,
                  emit_detached=True)
        def react_to_continue_decision(ctx: rs.ContextWrapper):
            rand = random.random()
            if ctx[nlp.prop_yesno].yes():
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_positive_expressions") + " Let's continue then"
                if rand < 0.7:
                    ctx[prop_emotion_publisher] = String(data=LUCKY_EMOTION)
                return rs.Emit()
            elif ctx[nlp.prop_yesno].no():
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_no_continuation")
                if rand < 0.7:
                    ctx[prop_emotion_publisher] = String(data=KISS_EMOTION)
                ctx[prop_game_in_progress] = False
                ctx[prop_stop_game] = True
            else:
                ctx[rawio.prop_out] = verbaliser.get_random_phrase(
                    "charades_misunderstanding")
                ctx[prop_continuation_unclear] = True


else:
    print("ros is not available")
