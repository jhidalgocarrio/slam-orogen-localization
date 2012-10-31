require 'eigen'

static_transform Eigen::Quaternion.from_euler(Eigen::Vector3.new(Math::PI, -14.034 * (Math::PI/180.00), 0 ), 2,1,0),
    Eigen::Vector3.new( -0.01302, 0.0654, -0.02794 ),
    "stim300" => "body"
