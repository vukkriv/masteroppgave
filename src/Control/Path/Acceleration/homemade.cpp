// ISO C++ 98 headers.
#include <cmath>
#include <queue>
#include <deque>

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Control
{
	namespace Path
	{
		namespace Acceleration
		{
			using DUNE_NAMESPACE;
			using std::queue;
			using std::deque;

			//DUNE Task Arguments
			struct Arguments
			{
				// Controller on/off
				bool use_controller;
				// Maximal acceleration 
				double max_acc; 

				// Controller gains; 
				double Kp;
				double Ki; 
				double Kd; 

				// Altitude on/off
				bool use_altitude;
				// Copter mass [kg]
				float copter_mass;
				//Wind drag coefficient 
				float wind_drag_coefficient; 
			}

			// Not sure what this is used for 
			static const std::string c_parcel_names[] = {DTR_RT("PID"), DTR_RT("Beta-X"),
                                                   DTR_RT("Beta-Y"), DTR_RT("Beta-Z"),
                                                   DTR_RT("Alpha45-Phi"), DTR_RT("Alpha45-Theta"), "Delayed-x", "Delayed-y", "PID-X", "PID-Y", "PID-Z", "ERROR", "ERROR-X", "ERROR-Y", "ERROR-Z"};
		    enum Parcel 
		    {
		    	PC_PID = 0,
		        PC_BETA_X = 1,
		        PC_BETA_Y = 2,
		        PC_BETA_Z = 3,
		        PC_ALPHA45_PHI = 4,
		        PC_ALPHA45_THETA = 5,
		        PC_DELAYED_X = 6,
		        PC_DELAYED_Y = 7,
		        PC_PID_X = 8,
		        PC_PID_Y = 9,
		        PC_PID_Z = 10,
		        PC_ERROR = 11,
		        PC_ERROR_X = 12,
		        PC_ERROR_Y = 13,
		        PC_ERROR_Z = 14
		    };

		    // Number of parcels
		    static const int NUM_PARCELS = 15;

		    // List controller types
		    enum ControllerType 
		    {
		        CT_PID,
		        CT_PID_INPUT,
		        CT_SUSPENDED,
		        CT_SUSPENDED_INPUT
		    };

		    // Angle measurements in NED
	      	class LoadAngle
	      	{
	      	public:
	        	double phi;			// Roll	
	        	double theta;		// Pitch
	        	double dphi;		// Roll derivative
	        	double dtheta;		// Pitch derivative
	        	double timestamp;
	        	LoadAngle(): phi(0.0), theta(0.0), dphi(0.0), dtheta(0.0), timestamp(0.0){};
	      	}; 

	      	struct Task: public DUNE::Control::PathController
	      	{
	      		//Task arguments
	      		Arguments m_args;

				//IMC messages 

	      		//Desired cointrol message 
	      		IMC::DesiredControl m_desired_control;
	      		//Autopilot mode message (Guided, Manual, Auto...) 
	      		IMC::AutopilotMode m_autopilot_mode;
	      		//Translational setpoint for logging
	      		IMC::DesiredLinearState m_setpoint_log;
	      		//Last estimated state recieved
	      		IMC::EstimatedState m_estimated_state; //Consider m_estate as name, used in Kristian Klausen's code 
	      		//Parcel Control Array 
	      		IMC::ControlParcel m_parcels[NUM_PARCELS];
	      		//Euler angles message 
	      		IMC::EulerAngles m_euler_angles;		//m_calculated angles 
	      		//Current desired speed
	      		IMC::DesiredSpeed m_dspeed;
	      		//Current desired altitude 
	      		IMC::DesiredZ m_dz;
	      		//Log history message
	      		IMC::RelativeState m_log; 

	      		//Reference model
	      		ReferenceModel m_refmodel; //Not sure if needed 
	      		//Current integrator value
	      		Matrix m_integrator_value; 
	      		//Timestamp of previous step
	      		double m_timestamp_prev_step; 
	      		//Reference history for input shaper
		        queue<ReferenceHistoryContainer> m_refhistory;
		        //Input shaper preferences
		        InputFilterConfig m_input_cfg;
		        //Last angle measurements 
		        LoadAngle m_loadAngle;

		        //MassMatrix and stuff
        		double m_massMatrix[25];
       			double m_coreolisMatrix[25];
        		Matrix m_MassMatrix;
        		Matrix m_CoreolisMatrix;
        		Matrix m_Gravity;
        		Matrix m_alpha_45;

        		//AngleHistory
		        deque<LoadAngleHistoryContainer> m_anglehistory;
		        Matrix m_delayed_feedback_desired_pos;
		        //Delayed Feedback State
		        DelayedFeedbackState m_delayed_feedback_state                 ;
		        //Shaping Filter State
		        InputShapingFilterState m_input_shaping_state;
		        //The current reference
		        Reference m_reference;
		        // Previous controller output
       			Matrix m_prev_controller_output

		        Task(const std::string& name, Tasks::Context& ctx):
		         DUNE::Control::PathController(name, ctx),
		         m_integrator_value(3,1, 0.0),
		         m_timestamp_prev_step(0.0),
		         m_prev_controller_output(3,1, 0.0),
		         m_MassMatrix(5,5, 0.0),
		         m_Gravity(5,5, 0.0),
		         m_alpha_45(2,1, 0.0),
		         m_delayed_feedback_desired_pos(3,1,0.0)
		        {
		        	param("Acceleration Controller", m_args.use_controller)
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.scope(Tasks::Parameter::SCOPE_MANEUVER)
		          	.defaultValue("false")
		          	.description("Enable Acc Controller NJIIIHAAAA");

		          	param("Max Acceleration", m_args.max_acc)
		          	.defaultValue("7")
		          	.units(Units::MeterPerSquareSecond)
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.description("Max acceleration of the vehicle");

		   			param("Controller - Kp", m_args.Kp)
		          	.units(Units::None)
		          	.defaultValue("1.152")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.description("P-Gain of the velocity controller");

		          	param("Controller - Kd", m_args.Kd)
		          	.units(Units::None)
		          	.defaultValue("2.24")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.description("D-Gain of the velocity controller");

		          	param("Controller - Ki", m_args.Ki)
		          	.units(Units::None)
		          	.defaultValue("0.08064")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.description("I-Gain of the velocity controller");

		          	param("Controller - Use Altitude", m_args.use_altitude)
		          	.defaultValue("true")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.description("Choose whether altitude is controlled or not (set to 0 if not).");

		          	param("Model - Copter Mass", m_args.copter_mass_kg)
		          	.defaultValue("3")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.units(Units::Kilogram)
		          	.description("Mass of the copter");

		         	param("Model - Wind Drag Coefficient", m_args.wind_drag_coefficient)
		          	.defaultValue("0.2")
		          	.visibility(Tasks::Parameter::VISIBILITY_USER)
		          	.scope(Tasks::Parameter::SCOPE_PLAN)
		          	.description("Coefficient to use in wind ff");

		          	//Bind functions

		          	bind<IMC::EulerAngles>(this);
          			bind<IMC::EstimatedState>(this);
          			bind<IMC::DesiredSpeed>(this);
          			bind<IMC::DesiredZ>(this);
        
		        


		        }

		        //Function for getting current state
				void consume(const IMC::EstimatedState* estate)
				{
					m_estate = *estate;
				}



                 













	      	} 






		}
	}
}