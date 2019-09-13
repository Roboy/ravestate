```
  _____     ____     _____   __ 
 |  __ \   / __ \   / ____| /_ |
 | |__) | | |  | | | (___    | |
 |  _  /  | |  | |  \___ \   | |
 | | \ \  | |__| |  ____) |  | |
 |_|  \_\  \____/  |_____/   |_|                                                              

```

## ROS1

The ROS1 module enables easy integration of Publishers, Subscribers and Service Calls into Ravestate.
It provides 3 Subclasses of property: Ros1SubProperty, Ros1PubProperty and Ros1CallProperty


### Using the Properties

#### Ros1SubProperty
Subclass of property that is synchronized with a given ROS1-Topic.  
Whenever it receives a new message from the topic, 
the value is written to the property and a changed-Signal is emitted.

Example: Property that subscribes to the ROS topic *chatter* of type *std_msgs/String* 
and state that reacts upon new messages

```python
import ravestate as rs
import ravestate_rawio as rawio
from ravestate_ros1 import Ros1SubProperty

from std_msgs.msg import String  # Import ROS message type

with rs.Module(name="ros_chatter"):

    prop_subscriber = Ros1SubProperty(name="subscriber", topic='chatter', msg_type=String)

    @rs.state(read=prop_subscriber, write=rawio.prop_out)
    def react_to_message(ctx: rs.ContextWrapper):
        message = ctx[prop_subscriber.changed()]  # message is of type std_msgs.msg.String
        ctx[rawio.prop_out] = f"Received on ROS-topic chatter: {message.data}"

```

#### Ros1PubProperty
Subclass of property that publishes all values written to it to a given ROS1-Topic.

Example: Property that publishes to the ROS topic *chatter* of type *std_msgs/String* 
and state that writes all outputs of the dialog system to the property

```python
import ravestate as rs
import ravestate_rawio as rawio
from ravestate_ros1 import Ros1PubProperty

from std_msgs.msg import String  # Import ROS message type

with rs.Module(name="ros_chatter"):

    prop_publisher = Ros1PubProperty(name="publisher", topic='chatter', msg_type=String)

    @rs.state(read=rawio.prop_out, write=prop_publisher)
    def ros_output(ctx: rs.ContextWrapper):
        # create message of type std_msgs.msg.String
        message = String(data=ctx[rawio.prop_out.changed()])
        ctx[prop_publisher] = message

```

#### Ros1CallProperty
Subclass of property that calls a ROS-Service when a value is written to it, 
blocks until it gets the response, writes back the response into the property and returns.

1. Build Request, can be of different form (see example):
    - Parameters as a dict
    - Parameters as ordered sequence
    - Matching Request-Type
2. Write Request to property, this blocks until a response is received and written back into the property.  
If no response is received (timeout is 10 seconds per default, can be set with call_timeout),
None is written back into the property.
3. Now the response can be read from the property. Check if it is None because service was unavailable!

Example: Property connected to the ROS service */add_two_ints* of type *rospy_tutorials/AddTwoInts* 
and state that calls the Service through the property when the input is "add"

```python
import ravestate as rs
import ravestate_rawio as rawio
from ravestate_ros1 import Ros1CallProperty

# Import ROS service type
from rospy_tutorials.srv import AddTwoInts, AddTwoIntsRequest


with rs.Module(name="add_two_ints"):

    prop_addtwoints = Ros1CallProperty(name="addtwoints", 
                                       service_name="/add_two_ints",
                                       service_type=AddTwoInts,
                                       call_timeout=5.0)

    # prop_addtwoints has to be in read and write
    @rs.state(cond=rawio.prop_in.changed(),
              read=(rawio.prop_in, prop_addtwoints),
              write=(rawio.prop_out, prop_addtwoints))
    def add_two_ints(ctx):
        if ctx[rawio.prop_in.changed()] == "add":
            # 1. Build Request
            # Option 1: Parameters as dict
            request = {'a': 11, 'b': 232}
            # Option 2: Parameters as ordered sequence
            request = (1, 2)
            # Option 3: Matching Request-Type
            request = AddTwoIntsRequest(1, 2)
            
            # 2. Write Request to property. 
            # This blocks until Response is written into property or service call timed out.
            ctx[prop_addtwoints] = request
            
            # 3. Read Response (of type rospy_tutorials.srv.AddTwoIntsResponse) from property
            response = ctx[prop_addtwoints]
            
            # Check if response is None
            if response:
                ctx[rawio.prop_out] = response.sum

```



#### Happy ROS-Ravestate Integration
