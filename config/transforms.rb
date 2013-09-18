require 'eigen'

#Transformation the translation is after performing the rotation (vbody = Tbody_stim300 * vstim300)
#static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI + (2.0*(Math::PI/180.00)), -14.654 * (Math::PI/180.00), 0 ), 2,1,0),
#static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -15.3 * (Math::PI/180.00), 0), 2,1,0),
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0 ), 2,1,0),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),#Eigen::Vector3.new( 0.01302, -0.0654, 0.02794 ),
    "stim300" => "body"

#Transformation when xsens is the imu
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, 14.034 * (Math::PI/180.00), 0 ), 2,1,0),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),#Eigen::Vector3.new( 0.01302, -0.0654, 0.02794 ),
    "xsens" => "body"

#Transformation the translation is after performing the rotation
# static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 3.9525 * (Math::PI/180.00)), 2,1,0),
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0.00), 2,1,0),
    Eigen::Vector3.new( 0.1406, 0.075, -0.21 ),
    "vicon_head" => "body"

#Transformation the translation is after performing the rotation
#static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(4.0 * (Math::PI/180.00), 0, 0 ), 2,1,0),
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(3.1* (Math::PI/180.00), 0, 0 ), 2,1,0),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "vicon_body" => "body"
