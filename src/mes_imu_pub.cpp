#include <RTIMULib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

int main( int argc, char **argv ) {
    ros::init( argc, argv, "mes_imu_node" );
    ROS_INFO( "Imu driver is now running" );
    ros::NodeHandle n;
    ros::NodeHandle private_n( "~" );

    std::string topic_name;
    if ( !private_n.getParam( "topic_name", topic_name ) ) {
        ROS_WARN( "No topic_name provided - default: imu/data" );
        topic_name = "imu/data";
    }
    else {
        ROS_INFO( "Success Get topic_name!" );
    }

    std::string calibration_file_path;
    if ( !private_n.getParam( "calibration_file_path", calibration_file_path ) ) {
        ROS_ERROR( "The calibration_file_path parameter must be set to use a calibration file." );
    }
    else {
        ROS_INFO( "Success Get calibration_file_path!" );
    }

    std::string calibration_file_name;
    if ( !private_n.getParam( "calibration_file_name", calibration_file_name ) ) {
        ROS_WARN( "No calibration_file_name provided - default: RTIMULib.ini" );
        calibration_file_name = "RTIMULib";
    }
    else {
        ROS_INFO( "Success Get calibration_file_name!" );
    }

    std::string frame_id;
    if ( !private_n.getParam( "frame_id", frame_id ) ) {
        ROS_WARN( "No frame_id provided - default: imu_link" );
        frame_id = "imu_link";
    }
    else {
        ROS_INFO( "Success Get frame_id!" );
    }

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>( topic_name.c_str(), 20 );

    // Load the RTIMULib.ini config file
    RTIMUSettings *settings = new RTIMUSettings( calibration_file_path.c_str(), calibration_file_name.c_str() );

    RTIMU *imu = RTIMU::createIMU( settings );

    if ( ( imu == NULL ) || ( imu->IMUType() == RTIMU_TYPE_NULL ) ) {
        ROS_ERROR( "No Imu found" );
        return -1;
    }
    else {
        ROS_INFO( "Found Imu!" );
    }

    // Initialise the imu object
    imu->IMUInit();

    // Set the Fusion coefficient
    imu->setSlerpPower( 0.02 );
    // Enable the sensors
    imu->setGyroEnable( true );
    imu->setAccelEnable( true );
    imu->setCompassEnable( true );

    ros::Rate loop_rate( 1000 / imu->IMUGetPollInterval() );
    ROS_INFO( "Get in while loop" );
    while ( ros::ok() ) {
        sensor_msgs::Imu imu_msg;

        if ( imu->IMURead() ) {
            RTIMU_DATA imu_data = imu->getIMUData();
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;
            imu_msg.orientation.x = imu_data.fusionQPose.x();
            imu_msg.orientation.y = imu_data.fusionQPose.y();
            imu_msg.orientation.z = imu_data.fusionQPose.z();
            imu_msg.orientation.w = imu_data.fusionQPose.scalar();
            imu_msg.angular_velocity.x = imu_data.gyro.x();
            imu_msg.angular_velocity.y = imu_data.gyro.y();
            imu_msg.angular_velocity.z = imu_data.gyro.z();
            imu_msg.linear_acceleration.x = imu_data.accel.x() * 9.81;
            imu_msg.linear_acceleration.y = imu_data.accel.y() * 9.81;
            imu_msg.linear_acceleration.z = imu_data.accel.z() * 9.81;

            imu_pub.publish( imu_msg );
        }

        ros::spinOnce();
    }
    return 0;
}