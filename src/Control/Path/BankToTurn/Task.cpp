
// Author: Marcus Frølich                                                   *

// DUNE headers.
#include <DUNE/DUNE.hpp>

using DUNE_NAMESPACES;

namespace Control
{
  namespace Path
  {
    namespace BankToTurn
    {
      struct Arguments
      {
        double K ;
        double Td;
        double Ti;
        double max_bank;
        double antiwindup_max;
        bool useCOG;
      };

      struct Task: public DUNE::Tasks::Task
      {
        IMC::DesiredRoll m_bank;

        //! Task arguments.
        Arguments m_args;

        double prev_time;
        double dt;

        double prev_error;
        double integral;
        double derivative;

        double heading;
        double d_heading;

        //	double time_straight_line_test;

        Task(const std::string& name, Tasks::Context& ctx):
          DUNE::Tasks::Task(name, ctx),
          prev_time(0.0),
          dt(0.0),
          prev_error(0.0),
          integral(0.0),
          derivative(0.0),

          heading(0.0),
          d_heading(-1.0)

        //		time_straight_line_test(0.0)
        {
          param("PID Gain Prop", m_args.K)
			    .defaultValue("1.2")
			    .description("Proportional gain");

          param("PID Gain Int", m_args.Ti)
          .defaultValue("0.1")
          .description("Integral gain");

          param("PID Gain Der", m_args.Td)
          .defaultValue("0.6")
          .description("Derivative gain");

          param("Maximum Bank", m_args.max_bank)
          .units(Units::Degree)
          .minimumValue("0.0")
          .maximumValue("90.0")
          .defaultValue("35.0")
          .description("Maximum Bank Angle");

          param("Anti Windup Max Bank", m_args.antiwindup_max)
          .units(Units::Degree)
          .minimumValue("0.0")
          .maximumValue("90.0")
          .defaultValue("3.0")
          .description("Maximum bank angle allowed for the integrator in PID");

          param("Use Course Over Ground", m_args.useCOG)
          .defaultValue("True")
          .description("Use course over ground or vehicle heading");

          bind<IMC::DesiredHeading>(this);
          bind<IMC::EstimatedState>(this);
          bind<IMC::ControlLoops>(this);
        }
        void
        consume(const IMC::ControlLoops* c_loops)
        {
          if ((c_loops->enable == IMC::ControlLoops::CL_ENABLE) &&
              (c_loops->mask & IMC::CL_YAW))
          {
            // Activate roll controller.
            IMC::ControlLoops cloops;
            cloops.enable = IMC::ControlLoops::CL_ENABLE;
            cloops.mask = IMC::CL_ROLL;
            dispatch(cloops);
            inf("ControlLoops CL_ROLL sent");
          }
        }

        void
        consume(const IMC::EstimatedState* es)
        {
          if(d_heading == -1)
            return;

          if(m_args.useCOG)	// Course over ground
            heading = Angles::degrees(atan2(es->vy,es->vx));
          else				// Vehicle heading
            heading = Angles::degrees(es->psi);

          PID();
        }

        void
        consume(const IMC::DesiredHeading* dh)
        {
          if(resolveEntity(dh->getSourceEntity()) == "Path Control"){
            d_heading = Angles::degrees(dh->value);
          }
        }

        void
        PID()
        {
          // Calculation of delta time (dt)
          double now = Clock().get();
          if(prev_time == 0)
            dt = 0;
          else
            dt = (now - prev_time);
          prev_time = now;

          if(dt < 0.001)
            return;


          // ONLY FOR STRAIGHT LINE TESTING!!!
          //		if(time_straight_line_test == 0){
          //			inf("\n\n00000000\n");
          //			time_straight_line_test = now;
          //		}
          //		if((now-time_straight_line_test) < 30){
          //			d_heading		= -10;
          //			inf("less");
          //		}else{
          //			d_heading		= 0;
          //			inf("more");
          //		}


          double error            = Angles::degrees(Angles::normalizeRadian(Angles::radians(d_heading - heading)));

          if(prev_error != 0)
            derivative       	= (error - prev_error)/dt;

          // Anti Windup
          if(m_args.Ti != 0){
            double pd				= m_args.K*(error + m_args.Td*derivative);
            double diff				= std::abs(pd) - std::abs(m_args.max_bank);
            if(diff > 0){
              // pd-effect is larger than max_bank
              integral			= 0;
            } else{
              // pd-effect is smaller than max_bank
              integral              	= integral + error*dt;
            }
            // Limits the impact of the integral term
            double anti_windup_max 	= m_args.antiwindup_max/(m_args.K*m_args.Ti);
            integral				= trimValue(integral,-anti_windup_max,anti_windup_max);
          }

          double bank_desired		= m_args.K*(error + m_args.Ti*integral + m_args.Td*derivative);
          prev_error              = error;

          double bank_demanded	= trimValue(bank_desired,-m_args.max_bank,m_args.max_bank);

          //inf("integral effect after = %f, derivative effect = %f", integral*m_args.Ti*m_args.K, m_args.K*m_args.Td*derivative);
          //inf("bank_demanded = %f, error = %f, integral = %f, derivative = %f",bank_demanded,error,integral*m_args.Ti*m_args.K,m_args.K*m_args.Td*derivative);
          //inf("bank_demanded = %f, error = %f, integral effect = %f, derivative effect = %f",bank_demanded,error,integral*m_args.Ti*m_args.K);

          m_bank.value = Angles::radians(bank_demanded);
          dispatch(m_bank);
        }

        //! Main loop.
        void
        onMain(void)
        {

          while (!stopping())
          {
            waitForMessages(1.0);
          }
        }
      };
    }
  }
}

DUNE_TASK
