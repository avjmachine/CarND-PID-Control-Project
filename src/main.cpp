#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  double Kp = 0.0;
  double Ki = 0.0;
  double Kd = 0.0;
  
  pid.Init(Kp, Ki, Kd);

  int no_steps = 200;
  int counter = -199;
 
  double penalty = 0.0;

  double delKp = 1.0;
  double delKi = 1.0;
  double delKd = 1.0;

  double error = 0.0;
  double best_error = 0.0;

  int knob_no = 1;

  h.onMessage([&pid, &Kp, &Ki, &Kd, &delKp, &delKi, &delKd, &no_steps, &counter, &penalty, &knob_no, &error, &best_error](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, 
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<string>());
          double speed = std::stod(j[1]["speed"].get<string>());
          double angle = std::stod(j[1]["steering_angle"].get<string>());
          double steer_value;
	  
         
	  if (cte > 5.4) {
              penalty += 5.4;
          }

	  if (cte<-5.4) {
	      penalty -= 5.4;
	  }
	  
	  if (counter <= 0) {
	      if (counter > (-1*no_steps/2)) {
	      error += (cte + penalty)*(cte + 5.4*penalty);
	      }
	  }

	  else if (counter > no_steps/2) {
	      if (cte > 5.4) {
	          penalty += 5.4;
	      }
	      if (cte < -5.4) {
	          penalty -= 5.4;
	      }
	      error += (cte + penalty)*(cte + penalty);
	  }

	  pid.Init(Kp, Ki, Kd);
	  pid.UpdateError(cte);
          steer_value = pid.TotalError(); 
          
	  if(counter == 0) {
	      best_error = error/(no_steps/2);
	      std::cout<<"\n"<<counter<<" Kp= "<<Kp<<" Ki= "<<Ki<<" Kd= "<<Kd<<"\n";
	      std::cout<<"\n"<<"Avg. Error: "<<best_error<<"\n";
	      error = 0.0;
	      penalty = 0;
	  }

	  else if (counter == 1) {
	      switch(knob_no){
		  case 1: Kp += delKp;
			  break;
		  case 2: Kp -= 2*delKp;
			  break;
	          case 3: Ki += delKi;
			  break;
	          case 4: Ki -= 2*delKi;
			  break;
	          case 5: Kd += delKd;
			  break;
		  case 6: Kd -= 2*delKd;
			  break;
	      }
	  }

	  else if (counter == no_steps) {
	      error /= (no_steps/2);
	      std::cout<<"\n"<<counter<<" Kp= "<<Kp<<" Ki= "<<Ki<<" Kd= "<<Kd<<"\n";
	      switch(knob_no) {
	          case 1: if (error < best_error) {
		              delKp *= 1.1;
	                      best_error = error;
			      knob_no = 3;
	                  }
			  else {
			      knob_no = 2;
			  }
			  break;
                  case 2: if (error < best_error) {
                              delKp *= 1.1;
                              best_error = error;
                          }
                          else {
			      Kp += delKp;
			      delKp *= 0.9;
                          }  
                          knob_no = 3;
			  break;
		  case 3: if (error < best_error) {
                              delKi *= 1.1;
                              best_error = error;
			      knob_no = 5;
                          }
                          else {
                              knob_no = 4;
                          }
                          break;
                  case 4: if (error < best_error) {
                              delKi *= 1.1;
                              best_error = error;
                          }
                          else {
                              Ki += delKi;
                              delKi *= 0.9;
                          }
                          knob_no = 5;
			  break;
		  case 5: if (error < best_error) {
                              delKd *= 1.1;
                              best_error = error;
                              knob_no = 1;
                          }
                          else {
                              knob_no = 6;
                          }
                          break;
                  case 6: if (error < best_error) {
                              delKd *= 1.1;
                              best_error = error;
                          }
                          else { 
                              Kd += delKd;
                              delKd *= 0.9;
                          }
                          knob_no = 1;	  
			  break;
	      }
	  std::cout<<"\n"<<"Avg. Error: "<<error<<"\n";
	  counter = 0;
	  penalty = 0;
	  }

	  // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value 
          //          << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
	  if ((fabs(cte) > 5.4) || (counter == 0)) {
	      msg = "42[\"reset\",{}]";
	  }
          //std::cout << msg << std::endl;
          counter += 1;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}
