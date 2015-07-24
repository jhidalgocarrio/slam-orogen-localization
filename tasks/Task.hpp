/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef LOCALIZATION_TASK_TASK_HPP
#define LOCALIZATION_TASK_TASK_HPP

#include "localization/TaskBase.hpp"

/** Framework Library dependencies includes **/
#include <localization/filters/Msckf.hpp> /** MSCKF class with Manifolds */
#include <localization/filters/MtkWrap.hpp> /** MSCKF wrapper for the state vector */
#include <localization/filters/State.hpp> /** Filter State */
//#include <localization/filters/ProcessModels.hpp> /** Filter Process Models */
//#include <localization/filters/MeasurementModels.hpp> /** Filters Measurement Models */

/** STD **/
#include <vector>
#include <cstdlib>

/** Eigen **/
#include <Eigen/Core> /** Core */
#include <Eigen/StdVector> /** For STL container with Eigen types **/

/** Envire **/
#include <envire_core/ItemBase.hpp>
#include <envire_core/Item.hpp>
#include <envire_core/LabeledTransformTree.hpp>
#include <envire_core/GraphViz.hpp>

/** Boost **/
#include <boost/lexical_cast.hpp> /** to string conversion when using < C++11 */
#include <boost/shared_ptr.hpp> /** For shared pointers **/
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>


namespace localization {

    /** Wrap the Multi and Single State **/
    typedef localization::MtkWrap<localization::State> WSingleState;
    typedef localization::MtkMultiStateWrap< localization::MultiState<localization::SensorState> > WMultiState;

    /** Filter and covariances types **/
    typedef localization::Msckf<WMultiState, WSingleState> MultiStateFilter;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> MultiStateCovariance;
    typedef ::MTK::vect<Eigen::Dynamic, double> MeasurementType;

    class MeasurementItem: public envire::core::Item<FeatureMeasurement>
    {
    };



    /*! \class Task 
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Exteroceptive update samples to correct the prediction (Visual, ICP, etc..).
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','localization::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument. 
     */
    class Task : public TaskBase
    {
	friend class TaskBase;

    protected:

        /******************************/
        /*** Control Flow Variables ***/
        /******************************/
        bool initFilter;

        /**************************/
        /*** Property Variables ***/
        /**************************/

        /******************************************/
        /*** General Internal Storage Variables ***/
        /******************************************/

        /** The filter uses by the Back-End **/
        boost::shared_ptr<MultiStateFilter> filter;

        /** Envire Tree **/
        envire::core::LabeledTransformTree envire_tree;

        /** Camera pose index in filter and envire **/
        std::vector<std::string> camera_node_labels;

        /**************************/
        /** Input port variables **/
        /**************************/

        /** Pose estimation **/
        ::base::samples::BodyState delta_pose;

        /***************************/
        /** Output port variables **/
        /***************************/
        base::samples::RigidBodyState pose_out;
        //base::samples::BodyState body_pose_out;

    protected:

        virtual void delta_pose_samplesTransformerCallback(const base::Time &ts, const ::base::samples::BodyState &delta_pose_samples_sample);

        virtual void visual_features_samplesTransformerCallback(const base::Time &ts, const ::localization::ExteroFeatures &visual_features_samples_sample);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "localization::Task");

        /** TaskContext constructor for Task 
         * \param name Name of the task. This name needs to be unique to make it identifiable for nameservices. 
         * \param engine The RTT Execution engine to be used for this task, which serialises the execution of all commands, programs, state machines and incoming events for a task. 
         * 
         */
        Task(std::string const& name, RTT::ExecutionEngine* engine);

        /** Default deconstructor of Task
         */
	    ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states. 
         *
         * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /**@brief Initialize the filter used in the Back-End
         */
        void initMultiStateFilter(boost::shared_ptr<MultiStateFilter> &filter, Eigen::Affine3d &tf);

        /** @brief Port out the values
        */
        void outputPortSamples(const base::Time &timestamp);

        localization::SensorState addSensorPoseToFilter(boost::shared_ptr<MultiStateFilter> filter, Eigen::Affine3d &tf);

        unsigned int removeSensorPoseFromFilter(boost::shared_ptr<MultiStateFilter> filter);

        void addMeasurementToEnvire(envire::core::LabeledTransformTree &envire_tree,
                            const std::string &camera_pose_label,
                            const ::localization::SensorState &camera_pose,
                            const ::localization::ExteroFeatures &samples);

        void removeSensorPoseFromEnvire(envire::core::LabeledTransformTree &envire_tree, const unsigned int it_removed_pose);

    public:
        static void removeRow(Eigen::MatrixXd& matrix, unsigned int rowToRemove)
        {
            unsigned int numRows = matrix.rows()-1;
            unsigned int numCols = matrix.cols();

            if( rowToRemove < numRows )
                matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) =
                    matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

            matrix.conservativeResize(numRows,numCols);
        }

        static void removeColumn(Eigen::MatrixXd& matrix, unsigned int colToRemove)
        {
            unsigned int numRows = matrix.rows();
            unsigned int numCols = matrix.cols()-1;

            if( colToRemove < numCols )
                matrix.block(0,colToRemove,numRows,numCols-colToRemove) =
                    matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

            matrix.conservativeResize(numRows,numCols);
        }

        template <typename Derived>
        static inline Eigen::Matrix<typename Derived::Scalar, 3, 3> makeSkewSymmetric(const Eigen::MatrixBase<Derived>& v)
        {
            EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE(Derived,3)

            Eigen::Matrix<typename Derived::Scalar, 3, 3> out;
            out <<   0, -v[2],  v[1],
                  v[2],     0, -v[0],
                 -v[1],  v[0],     0;

            return out;
        }

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW


    };
}

#endif

