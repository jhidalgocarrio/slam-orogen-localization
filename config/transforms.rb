require 'eigen'

static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0 ), 2,1,0),
    Eigen::Vector3.new( 0.01302, -0.0654, 0.02794 ),
    "stim300" => "body"

static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI - 8.33 * (Math::PI/180.00), -14.034 * (Math::PI/180.00), 3.969 * (Math::PI/180.00)), 2,1,0),
    Eigen::Vector3.new( 0.09, -0.155, 0.195 ),
    "vicon_head" => "body"

static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(4.0 * (Math::PI/180.00), 0, 0 ), 2,1,0),
    Eigen::Vector3.new( 0.0, 0.0, 0.0 ),
    "vicon_body" => "body"
