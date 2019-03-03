```
  _______   _                                _____ ____  
 |__   __| | |                              |_   _/ __ \ 
    | | ___| | ___  __ _ _ __ __ _ _ __ ___   | || |  | |
    | |/ _ \ |/ _ \/ _` | '__/ _` | '_ ` _ \  | || |  | |
    | |  __/ |  __/ (_| | | | (_| | | | | | |_| || |__| |
    |_|\___|_|\___|\__, |_|  \__,_|_| |_| |_|_____\____/ 
                    __/ |                                
                   |___/                                 
                                                                    
```

## TelegramIO

The TelegramIO module enables ravestate to connect to a Telegram-Bot and chat to people there.
The connection to Telegram is managed with [python-telegram-bot](https://github.com/python-telegram-bot/python-telegram-bot)

### Architecture
There are two main modes for this module:
* Single-Process-Mode: All chats share the same context
* Multiprocess-Mode: Every chat creates its own context in a separate process

#### Single-Process-Mode
<p align="center">
  <img src="../../resources/docs/telegram_singleprocess.png" height="400">
</p>

In this mode the module handles incoming text messages and pictures from all chats.
Outgoing messages are sent to every currently active chat.

#### Multiprocess-Mode
<p align="center">
  <img src="../../resources/docs/telegram_multiprocess.png" height="250">
</p>

In this mode the "Master" part of the module is running in the main process of ravestate.

Whenever there is a new chat, a new instance of ravestate is started in a new process and 
a bidirectional pipe is set up to enable communication between the main process and the new child process.

Only the main process is connected to the Telegram-Bot and therefore any incoming messages get forwarded to the 
corresponding child process via the pipe.
The main process also forwards any incoming messages it receives through the pipe to the corresponding telegram chat.

In order to clean up unused child processes, the main process kills child processes after a configurable amount of inactivity.

Child processes running the TelegramIO-Module listen for incoming text messages or pictures on the pipe
and write output to the pipe. They only exchange messages with a single chat (indirectly via the pipe).

### Abilities
The TelegramIO module is able to handle incoming text messages as well as incoming pictures.
The module creates an interlocutor node for every user it is chatting with,
containing the user's telegram-id, username and full name if it is set by the user.

Incoming text messages are simply written into the RawIO Input property.

For incoming pictures, the picture is saved locally as a file and the filepath is written to the RawIO Pic_In Context property.

Messages in the RawIO Output are sent to the Telegram Chat(s).

### Configuration
There are 5 configurable parameters (see [__init__.py](__init__.py)):
* _telegram-token_: Put the Token of your Telegram-Bot here
* _all_in_one_context_: True if Single-Process-Mode should be used, False if Multiprocess-Mode should be used.
* _child_conn_: Not to be set in a config file or via the command line. Will be set by master process as a runtime_override.
* _child_config_files_: If in Multiprocess-Mode, the config-paths listed here will be used when creating a new context for a new chat.
* _chat_lifetime_: The timespan in minutes in which a chat will be kept active after the last message
