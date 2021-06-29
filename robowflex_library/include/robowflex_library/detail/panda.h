/* Author: Zachary Kingston */

#ifndef ROBOWFLEX_PANDA_
#define ROBOWFLEX_PANDA_

#include <robowflex_library/robot.h>
#include <robowflex_library/planning.h>

namespace robowflex
{
    /** \cond IGNORE */
    ROBOWFLEX_CLASS_FORWARD(PANDARobot);
    /* \endcond */

    /** \class robowflex::PANDARobotPtr
        \brief A shared pointer wrapper for robowflex::PANDARobot. */

    /** \class robowflex::PANDARobotConstPtr
        \brief A const shared pointer wrapper for robowflex::PANDARobot. */

    /** \brief Convenience class that describes the default setup for PANDA. Will first attempt to load
     * configuration and description from the robowflex_resources package. See
     * https://github.com/KavrakiLab/robowflex_resources for this package. If this package is not available,
     * then franka_description will be used.
     */
    class PANDARobot : public Robot
    {
    public:
        /** \brief Constructor.
         */
        PANDARobot();

        /** \brief Initialize the robot with manipulator kinematics.
         *  \return True on success, false on failure.
         */
        bool initialize();

    private:
        static const std::string DEFAULT_URDF;        ///< Default URDF
        static const std::string DEFAULT_SRDF;        ///< Default SRDF
        static const std::string DEFAULT_LIMITS;      ///< Default Limits
        static const std::string DEFAULT_KINEMATICS;  ///< Default kinematics

        static const std::string RESOURCE_URDF;        ///< URDF from robowflex_resources
        static const std::string RESOURCE_SRDF;        ///< SRDF from robowflex_resources
        static const std::string RESOURCE_LIMITS;      ///< Limits from robowflex_resources
        static const std::string RESOURCE_KINEMATICS;  ///< kinematics from robowflex_resources
    };

    namespace OMPL
    {
        /** \cond IGNORE */
        ROBOWFLEX_CLASS_FORWARD(PANDAOMPLPipelinePlanner);
        /* \endcond */

        /** \class robowflex::OMPL::PANDAOMPLPipelinePlannerPtr
            \brief A shared pointer wrapper for robowflex::OMPL::PANDAOMPLPipelinePlanner. */

        /** \class robowflex::OMPL::PANDAOMPLPipelinePlannerConstPtr
            \brief A const shared pointer wrapper for robowflex::OMPL::PANDAOMPLPipelinePlanner. */

        /** \brief Convenience class for the default motion planning pipeline for PANDA.
         */
        class PANDAOMPLPipelinePlanner : public OMPLPipelinePlanner
        {
        public:
            /** \brief Constructor.
             *  \param[in] robot Robot to create planner for.
             *  \param[in] name Namespace of this planner.
             */
            PANDAOMPLPipelinePlanner(const RobotPtr &robot, const std::string &name = "");

            /** \brief Initialize the planning context. All parameter provided are defaults.
             *  \param[in] settings Settings to set on the parameter server.
             *  \param[in] adapters Planning adapters to load.
             *  \return True on success, false on failure.
             */
            bool initialize(const Settings &settings = Settings(),
                            const std::vector<std::string> &adapters = DEFAULT_ADAPTERS);

        private:
            static const std::string DEFAULT_CONFIG;   ///< Default planning configuration.
            static const std::string RESOURCE_CONFIG;  ///< Planning configuration from robowflex_resources.
        };
    }  // namespace OMPL
}  // namespace robowflex

#endif
