```
(C) Roboy 2019            
       _     _                              
      | |   (_)              _              
 _____| |  _ _ ____  _____ _| |_ ___   ____ 
(____ | |_/ ) |  _ \(____ (_   _) _ \ / ___)
/ ___ |  _ (| | | | / ___ | | || |_| | |    
\_____|_| \_)_|_| |_\_____|  \__)___/|_|    
                                                                       
```

## 20 Questions with Akinator

The Ravestate Akinator module is now offering the 20 questions game.
It consists of a wrapper for the API calls to the  [Akinator](https://en.akinator.com/)  web game and several states to handle the game flow with the interlocutor.

### Architecture
This is an overview of the dialog flow for the 20 questions game. 
<img src="../resources/docs/Akinator.png" width="480" align="middle">

## Starting the Game
There are two possibilities to trigger the game:
1. Interlocutor inputs something like: "I want to play", "I like games" or something similar
2. Ravestate starts it automatically: When the system gets bored (no dialog states are active but an interlocutor is present) then Akinator is one of the possible modules that can be triggered. 