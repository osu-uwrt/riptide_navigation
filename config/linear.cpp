#include <ros/ros.h>
    #include <actionlib/server/simple_action_server.h>
    #include <actionlib_tutorials/LinearAction.h>
    
    class LinearAction
    {
    protected:
    
      ros::NodeHandle nh_;
     actionlib::SimpleActionServer<actionlib_tutorials::LinearAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
     std::string action_name_;
     // create messages that are used to published feedback/result
     actionlib_tutorials::LinearFeedback feedback_;
     actionlib_tutorials::LinearResult result_;
   
   public:
   
   LinearAction(std::string name) :
     as_(nh_, name, boost::bind(&LinearAction::executeCB, this, _1), false),
       action_name_(name)
     {
       as_.start();
     }
   
     ~LinearAction(void)
     {
     }
   
     void executeCB(const actionlib_tutorials::LinearGoalConstPtr &goal)
     {
       // helper variables
       ros::Rate r(1);
       bool success = true;
   
       feedback_.x.clear();
       feedback_.y.clear();
       feedback_.z.clear();
       
   
       // publish info to the console for the user
    //    ROS_INFO("%s: Executing, creating fibonacci sequence of order %i with seeds %i, %i", action_name_.c_str(), goal->order, feedback_.sequence[0], feedback_.sequence[1]);
   
       // start executing the action
         // check that preempt has not been requested by the client
         if (as_.isPreemptRequested() || !ros::ok())
         {
           ROS_INFO("%s: Preempted", action_name_.c_str());
           // set the action state to preempted
           as_.setPreempted();
           success = false;
           break;
         }
       feedback_.x.push_back(newX);
       feedback_.y.push_back(newY);
       feedback_.z.push_back(newZ);

         // publish the feedback
        as_.publishFeedback(feedback_);
       // this sleep is not necessary, the sequence is computed at 1 Hz for demonstration purposes
         r.sleep();
       }
   
       if(success)
       {
         result_.x = feedback_.x;
         result_.y = feedback_.y;
         result_.z = feedback_.z;
         ROS_INFO("%s: Succeeded", action_name_.c_str());
       // set the action state to succeeded
         as_.setSucceeded(result_);
       }
     }
   
   
   };
   
   
   int main(int argc, char** argv)
   {
     ros::init(argc, argv, "linear");
   
     LinearAction linear("linear");
     ros::spin();
   
     return 0;
   }