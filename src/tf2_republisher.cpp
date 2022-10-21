//
// Created by waxz on 22-10-21.
//

#include <thread>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/transport_hints.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "my_tf2_listener");

    ros::NodeHandle node;


    std::vector<geometry_msgs::TransformStamped> static_tf_vector;

    std::set<std::string> frame_pair;


    std::mutex mtx;

    auto static_subscription_callback = [&](const ros::MessageEvent<tf2_msgs::TFMessage const> &msg_evt) {


        auto ts = msg_evt.getMessage();
//        ROS_INFO("receive static tf size = [%ld]", ts->transforms.size());

        for (auto &t: ts->transforms) {
            char tf_key[100];

            sprintf(tf_key, "%s-%s", t.header.frame_id.c_str(), t.child_frame_id.c_str());
//            ROS_INFO("receive static tf [%s]", tf_key);

            {
                std::lock_guard<std::mutex> locker(mtx);
                auto tf_key_str = std::string(tf_key);
                auto it = frame_pair.find(tf_key_str);
                if (it == frame_pair.end()) {

                    ROS_INFO("add static tf [%s]", tf_key);

                    frame_pair.insert(tf_key_str);
                    static_tf_vector.push_back(t);
                }
            }

        }
    };

    auto hints = ros::TransportHints()
            .unreliable()
            .reliable()
            .maxDatagramSize(1000)
            .tcpNoDelay();


#if 0
    ros::Subscriber sub = node.subscribe<tf2_msgs::TFMessage>("/tf_static", 1000,
                                                              static_subscription_callback, nullptr, hints );
#endif

    ros::NodeHandle n_a;

    ros::CallbackQueue callback_queue_a;
    n_a.setCallbackQueue(&callback_queue_a);


    ros::SubscribeOptions ops;
    ops.transport_hints = hints;

    ops.template init<tf2_msgs::TFMessage>("/tf_static", 100,
                                           static_subscription_callback);
    ops.transport_hints = hints;

    ops.allow_concurrent_callbacks = true;
    ros::Subscriber sub_a = n_a.subscribe(ops);

    std::thread spinner_thread_a([&]() {
        ros::MultiThreadedSpinner spinner(4);
        spinner.spin(&callback_queue_a);
    });


#if 0
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
#endif

    tf2_ros::StaticTransformBroadcaster static_broadcaster;


    tf::TransformBroadcaster tf_br;
    tf::StampedTransform pub_tf_stamped;
    ros::Duration transform_tolerance_;
    transform_tolerance_.fromSec(1.0);


    ros::Rate rate(1);
    geometry_msgs::TransformStamped static_transformStamped;

#if 0
    geometry_msgs::TransformStamped transformStamped;
        try{
            transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1",
                                                        ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_INFO("%s",ex.what());
        }
#endif

    while (node.ok()) {
        ros::spinOnce();
        auto time = ros::Time::now();
        {
            std::lock_guard<std::mutex> locker(mtx);
            for (auto &t: static_tf_vector) {
                t.header.stamp = time;
                static_broadcaster.sendTransform(t);
#if 0
                ros::Time transform_expiration = (time +
                                              transform_tolerance_);

            pub_tf_stamped.frame_id_ = t.header.frame_id;
            pub_tf_stamped.child_frame_id_ = t.child_frame_id;

            pub_tf_stamped.setOrigin(tf::Vector3( t.transform.translation.x, t.transform.translation.y,t.transform.translation.z  ));
            pub_tf_stamped.setRotation(tf::Quaternion(t.transform.rotation.x, t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w ));

            pub_tf_stamped.stamp_ = transform_expiration;

            tf_br.sendTransform(pub_tf_stamped);
#endif
            }

        }

        rate.sleep();
    }
    spinner_thread_a.join();

    return 0;
};