# The Human Bot-Human Interaction Tutorial

## The Building Blocks

For building your interactive dialogue app, Ravestate gives you three
basic building blocks to work with: __Properties__,
__Signals__ and __States__. Their roles are straight-forward:

__Properties__ are global variables that can be read and manipulated
by states. If you think in terms of traditional slot-filling dialogue
apps, properties are the slots. For example, a flight-booking dialogue
app would try to fill user-specified `Origin` and `Destination` properties.
	
```
 ╭───────────╮    ┌───────>──────┐    ╭────────────╮
 │ Raw Input │ -> │ Dialogue App │ -> │ Raw Output │
 ╰───────────╯    └───────<──────┘    ╰────────────╯
                   Write 🠛 🠙 Read
         ╭────────────────────────────────╮
         │ Application properties / Slots │
         ╰────────────────────────────────╯
```

**NOTE:** Ravestate properties assume a broader role. They are not
just used to keep track of application data, they also hold *system input and output*.
In a natural language dialogue system, the most important input is an
interlocutor (user) utterance, and the most important output is the textual
response to this input. These central I/O properties are called **raw input** and
**raw output**.

If properties represent data, then __states__ represent *processes*. These processes
are triggered by specific conditions in your application (e.g. a user might call
your app a __"sexy cactus"__). Reacting to such events, states
have various superpowers at their disposal:
 * They can read and write properties like user utterance/response.
 * They can emit __signals__ (this is where it gets interesting!).
 * They can create new states and properties (this is where it gets a little bit spooky).

It is safe to say that few dialogue systems allow this. The state
machine (be it traditional or neural) that drives your dialogue app is usually
seen as static during runtime. In Ravestate, it is very much not.

The final building block is __Signals__. They are what glues your
properties (data) and states (processes) together. A __Signal__
represents an *event*, such as ...
 * *... a property value changed!*, or
 * *... the user utterance is a question!*, or
 * *... you must construct additional pylons."*

Signals are emitted by properties when their value is changed, or by states
with a Ravestate API call. __They are received by states, which constrain
their execution to the presence of specific signals.__

```            
              [Write]
 ╭────────────<-----┌────────┐  [Activate other states with signals
 │ Properties │     │ States │!⮌ emitted during state exec.]
 ╰────────────╯--!-->────────┘
  [Activate states with changed-signals]
```

**Here is the catch:** States may use a boolean combination
of signals as their execution constraint. For example, the following
would be perfectly reasonable:
* `input-is-question` **AND** `input-is-personal`
  🠚 The input is probably a personal question.
* `what-is-this-asked` **AND** `object-recognized`
  🠚 We were asked to recognize an object and found one.
* (`input-is-question` **AND** `input-about-trains`) **OR** `bored`
  🠚 Maybe now is a good moment to remark on the likability of trains.

## Hello World

A simple patterned response.

## Modules

## Fallback

How specificity guides activation priority.

## Question Answering

The question-asked signal.

The question-answered signal.

The response state.

A timeout handler.

## Loops

Detaching from the causal chain.

## I/O modules: Who writes the input?

## Dynamically created states 

Attaching things to new interlocutors