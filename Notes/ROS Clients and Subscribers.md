# ROS Clients and Subscribers

Writing the `arm_mover node`, you practiced generating custom messages, publishing to a topic, building ROS services servers, setting parameters, and creating launch files. You almost have a complete overview of ROS, but you still have to learn ROS **clients** to request services from client nodes, as well as ROS **subscribers**.

## ROS Clients

A service client defined inside a service client node can request services from a service server node. In C++, ROS clients frequently have the following format, although other parameters and arguments are possible:

```C++
ros::ServiceClient client = n.serviceClient<package_name::service_file_name>("service_name");
```

The `client` object is instantiated from the ros::ServiceClient class. This object allows you to request services by calling the `client.call()` function.

To communicate with the ROS Master in C++, you need a **NodeHandle**. The node handle `n` will initialize the node.

The `package_name::service_file_name` indicates the name of the service file located in the `srv`directory of the package.

The `service_name` argument indicates the name of the service which is defined in the service server node.

## ROS Subscribers

A subscriber enables your node to read messages from a topic, allowing useful data to be streamed to the node. In C++, ROS subscribers frequently have the following format, although other parameters and arguments are possible:

```C++
ros::Subscriber sub1 = n.subscribe("/topic_name", queue_size, callback_function);
```

The `sub1` object is a subscriber object instantiated from the ros::Subscriber class. This object allows you to subscribe to messages by calling the `subscribe()` function.

To communicate with the ROS Master in C++, you need a **NodeHandle**. The node handle `n` will initialize the node.

The `"/topic_name"` indicates the topic to which the Subscriber should listen.

The `queue_size` determines the number of messages that can be stored in a queue. If the number of messages published exceeds the size of the queue, the oldest messages are dropped. As an example, if the `queue_size` is set to 100 and the number of messages stored in the queue is equal to 100, we will have to start deleting old messages to make room in the queue for new messages. This means that we are unable to process messages fast enough and we probably need to increase the `queue_size`.

The `callback_function` is the name of the function that will be run each incoming message. Each time a message arrives, it is passed as an argument to `callback_function`. Typically, this function performs a useful action with the incoming data. Note that unlike service handler functions, the `callback_function` is not required to return anything.