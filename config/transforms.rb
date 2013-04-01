require 'eigen'

static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI + 2.0*(Math::PI/180.00), -14.034 * (Math::PI/180.00), 0 ), 2,1,0),
# static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0 ), 2,1,0),
    Eigen::Vector3.new( 0.01302, -0.0654, 0.02794 ),
    "stim300" => "body"

#In this tranformation the translation is after performing the rotation
# static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 3.9525 * (Math::PI/180.00)), 2,1,0),
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0.00), 2,1,0),
    Eigen::Vector3.new( 0.1406, 0.075, -0.21 ),
    "vicon_head" => "body"

#In this tranformation the translation is after performing the rotation
static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(4.0 * (Math::PI/180.00), 0, 0 ), 2,1,0),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "vicon_body" => "body"
