import ravestate as rs
import ravestate_rawio as rawio
import ravestate_conio
import ravestate_nlp as nlp
import ravestate_idle as idle

from reggol import get_logger
logger = get_logger(__name__)

ROS_AVAILABLE = False
try:
    import rospy
    from ha_recognition.srv import Test
    from ha_recognition.msg import Test2
    from ravestate_ros1 import Ros1SubProperty
    from ravestate_ros1 import Ros1CallProperty
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

if ROS_AVAILABLE:

    with rs.Module(name="charades", depends=(nlp.mod, idle.mod, rawio.mod)) as mod:

        # signals
        sig_mentioned_charades = rs.Signal("mentioned charades")
        sig_offered_charades = rs.Signal("offer_game")
        sig_play_decision_asked = rs.Signal("play_decision_done")
        sig_rules_decision_done = rs.Signal("rules_decision_done")
        sig_activity_choice_prompted = rs.Signal("activity_choice_propmted")
        sig_readiness_check_started = rs.Signal("readiness_check_started")
        sig_action_captured = rs.Signal("action_captured")
        sig_label_named = rs.Signal("action_captured")
        sig_round_completed = rs.Signal("round_completed")
        sig_asked_to_continue = rs.Signal("asked_to_continue")
        sig_continue_game = rs.Signal("continue_game")

        # properties

        prop_choice_unclear = rs.Property(name="prop_choice_unclear", always_signal_changed=True, default_value="")
        prop_activity_chosen = rs.Property(name="prop_activity_chosen", always_signal_changed=True, default_value="")
        prop_game_in_progress = rs.Property(name="prop_game_in_progress", always_signal_changed=True, default_value=False)
        prop_waiting_for_label = rs.Property(name="prop_waiting_for_label", always_signal_changed=True, default_value=False)


        # ros properties


        prop_record_action = Ros1CallProperty(name="_recognition",
         service_name="recognition", 
         service_type=Test, 
         call_timeout=5.0)
        prop_label_subscriber = Ros1SubProperty(name="label_subsriber", 
            topic="/predictor", 
            msg_type=Test2, 
            always_signal_changed=True)

        def recognition_client():
            rospy.wait_for_service("recognition")
            try:
                record = rospy.ServiceProxy("recognition", Test)
                resp = record()
                if resp:
                    return True
            except rospy.ServiceException as e:
                print('Service call failed: ',  e)


        # general states


        @rs.state(cond=nlp.prop_tokens.changed(), read=(nlp.prop_lemmas, prop_game_in_progress), signal=sig_mentioned_charades)
        def signal_charades(ctx: rs.ContextWrapper):
            '''
            reacts to mentioning charades with a sig_mentioned_charades signal
            '''
            if 'charade' in ctx[nlp.prop_lemmas] and not ctx[prop_game_in_progress]:
                return rs.Emit()

        @rs.state(cond=idle.sig_bored, write=(rawio.prop_out, prop_game_in_progress, prop_waiting_for_label))
        def stop_game_session(ctx: rs.ContextWrapper):
            '''
            go off the game session if idle, so that a new game can be trigerred if the previous session is abandoned
            '''
            ctx[prop_game_in_progress] = False
            ctx[prop_waiting_for_label] = False
            ctx[rawio.prop_out] = "Stopping game session"


        # main game convo flow

        @rs.state(cond=nlp.sig_intent_play | sig_mentioned_charades | (sig_play_decision_asked & prop_choice_unclear.changed()), 
            read=(nlp.prop_tokens, prop_game_in_progress), write=rawio.prop_out, signal=sig_offered_charades, emit_detached=True)
        def offer_game(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                ctx[rawio.prop_out] = "Would you like to play Charades?"
                return rs.Emit(wipe=True)
            else:
                rs.Resign()

        @rs.state(cond=nlp.prop_yesno.changed() & sig_offered_charades, read=(nlp.prop_yesno, prop_game_in_progress),
         write=(rawio.prop_out, prop_choice_unclear), signal=sig_play_decision_asked, emit_detached=True)
        def process_play_decision(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                if ctx[nlp.prop_yesno].yes():
                    ctx[rawio.prop_out] = "Cool. Do you want to hear the rules?"
                elif ctx[nlp.prop_yesno].no():
                    ctx[rawio.prop_out] = "Bad for you"
                    return rs.Resign()
                else:
                    ctx[prop_choice_unclear] = True
                    ctx[rawio.prop_out] = "I'm sorry. I didn't understand"
                return rs.Emit()
            else:
                return rs. Resign()     

        @rs.state(cond=nlp.prop_yesno.changed() & sig_play_decision_asked.max_age(10), read=(nlp.prop_yesno, prop_game_in_progress),
         write=(rawio.prop_out, prop_game_in_progress), signal=sig_rules_decision_done, emit_detached=True)
        def react_to_rules_decision(ctx: rs.ContextWrapper):
            if not ctx[prop_game_in_progress]:
                rules = '''Ok. So you pick an activity and demonstrate it to me and I try to guess what it is that you are showing. 
                    I know a lot of activities, but not all in the world. 
                    You can consult my tablet to pick one of the actions. Just click on it and you will see all the available actions
                    When I will actually start recording I will say Beep so you can start demonstrating the activity.
                    You will have about three seconds to show the activity to me until I say beep again.'''
                if ctx[nlp.prop_yesno].no():
                    out = "Ok, let's get to the game then"
                elif ctx[nlp.prop_yesno].yes():
                    out = rules
                else:
                    out = "I will count that as yes. " + rules
                ctx[rawio.prop_out] = out
                ctx[prop_game_in_progress] = True
                return rs.Emit()
            else:
                return rs.Resign()

        @rs.state(cond=sig_rules_decision_done.min_age(2) | sig_continue_game,
         write=rawio.prop_out, signal=sig_activity_choice_prompted, emit_detached=True)
        def offer_activity_choice(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = '''Please choose one activity that you will show to me. Once you choose it say start'''
            return rs.Emit()

        @rs.state(cond=nlp.prop_tokens.changed() & sig_activity_choice_prompted.max_age(15),
            read=nlp.prop_lemmas, write=(rawio.prop_out, prop_choice_unclear, prop_activity_chosen), 
            signal=sig_readiness_check_started, emit_detached=True)
        def react_to_choice(ctx: rs.ContextWrapper):
            if "start" in ctx[nlp.prop_lemmas]:
                ctx[rawio.prop_out] = "Great. Then prepare yourself"
                ctx[prop_activity_chosen] = True
            else:
                ctx[rawio.prop_out] = "I didn't get what you said"
                ctx[prop_choice_unclear] = True
            return rs.Emit()

        @rs.state(cond=sig_activity_choice_prompted.min_age(16).max_age(-1) | (sig_readiness_check_started & prop_choice_unclear.changed()),
            write=rawio.prop_out, signal=sig_activity_choice_prompted, emit_detached=True)
        def ping_actvity_choice(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = "Have you chosen anything? Say start when you're ready"
            return rs.Emit()

        @rs.state(cond=sig_readiness_check_started & prop_activity_chosen.changed().min_age(2), read=prop_record_action, 
            write=(rawio.prop_out, prop_waiting_for_label), signal=sig_action_captured, emit_detached = True)
        def start_recording(ctx: rs.ContextWrapper):
            # need to divide this state in two, it won't work that way
            ctx[rawio.prop_out] = '''Ready? One, two, three and Beep!'''
            # request = ()
            # ctx[prop_record_action] = request
            # response = ctx[prop_record_action]
            resp = recognition_client()
            if resp:
                ctx[rawio.prop_out] = "Beep! Now let me think a little bit"
                ctx[prop_waiting_for_label] = True
                return rs.Emit()
            else:
                ctx[rawio.prop_out] = BROKEN_MESSAGE

        @rs.state(cond=prop_label_subscriber.changed(), 
            read=(prop_label_subscriber, prop_waiting_for_label), 
            write=(rawio.prop_out, prop_waiting_for_label), signal=sig_label_named, emit_detached=True)
        def name_the_label(ctx: rs.ContextWrapper):
            # if ctx[prop_waiting_for_label]:
            message = ctx[prop_label_subscriber]
            if message:
                ctx[rawio.prop_out] = message
                ctx[prop_waiting_for_label] = False
                return rs.Emit()
            else:
                ctx[rawio.prop_out] = BROKEN_MESSAGE
            # else:
            #     return rs.Resign()


        @rs.state(cond=sig_action_captured.min_age(16).max_age(-1), 
            read=prop_waiting_for_label, write=rawio.prop_out)
        def no_prediction(ctx: rs.ContextWrapper):
            if ctx[prop_waiting_for_label]:
                ctx[rawio.prop_out] = BROKEN_MESSAGE


        @rs.state(cond=nlp.prop_yesno.changed() & (sig_label_named | prop_choice_unclear.changed()), 
            write=(rawio.prop_out, prop_choice_unclear), signal=sig_round_completed, emit_detached=True)
        def react_to_feedback(ctx: rs.ContextWrapper):
            if ctx[nlp.prop_yesno].yes():
                ctx[rawio.prop_out] = "Yay! I am so smart"
                return rs.Emit()
            elif ctx[nlp.prop_yesno].no():
                ctx[rawio.prop_out] = "Oh no! Silly me"
                return rs.Emit()
            else:
                ctx[rawio.prop_out] = "I did not understand what you said. Did I guess correctly?"
                ctx[prop_choice_unclear] = True
            

        @rs.state(cond=sig_round_completed, write=rawio.prop_out, signal=sig_asked_to_continue, emit_detached=True)
        def ask_to_continue(ctx: rs.ContextWrapper):
            ctx[rawio.prop_out] = "Would you like to play another round?"
            return rs.Emit()

        @rs.state(cond=nlp.prop_yesno.changed() & sig_asked_to_continue, write=(rawio.prop_out, prop_game_in_progress), 
            signal=sig_continue_game, emit_detached=True)
        def react_to_continue_decision(ctx: rs.ContextWrapper):
            if ctx[nlp.prop_yesno].yes():
                ctx[rawio.prop_out] = "Great. Let's continue then"
                return rs.Emit()
            elif ctx[nlp.prop_yesno].no():
                ctx[rawio.prop_out] = "Ok. Thanks for playing. It was fun"
                ctx[prop_game_in_progress] = False
            else:
                ctx[rawio.prop_out] = "I will count that as yes. Let's continue."
                return rs.Emit()

            
else:
    print("ros is not available")




