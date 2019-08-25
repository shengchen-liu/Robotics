# Pub-Sub Class

Inside the publisher and subscriber nodes of this lesson, global variables and objects were defined to be used anywhere in the code. We did this to simplify the code, but it is not a good practice. You should always write a pub-sub class to easily share variables and objects with any callback function in your code. Here’s a [ROS pub-sub template class](https://answers.ros.org/question/59725/publishing-to-a-topic-via-subscriber-callback-function/) that you can use:

## ROS Class C++ Code

```C++
#include <ros/ros.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<PUBLISHED_MESSAGE_TYPE>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const SUBSCRIBED_MESSAGE_TYPE& input)
  {
    PUBLISHED_MESSAGE_TYPE output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
```

We challenge you to use this template class to implement the nodes in this lesson.