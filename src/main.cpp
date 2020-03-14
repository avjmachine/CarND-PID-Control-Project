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
  double Kp = -0.19;
  double Ki = 0.0;
  double Kd = -1.0;
  
  pid.Init(Kp, Ki, Kd);

  int no_steps = 900;
  int counter = -899;
 
  double penalty = 0.0;

  double delKp = 0.01;
  double delKi = 0.001;
  double delKd = 0.01;

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

          
	  if(counter == 0) {
	      best_error = error/no_steps;
	      std::cout<<counter<<" Kp= "<<Kp<<" Ki= "<<Ki<<" Kd= "<<Kd<<"Avg. Error: "<<best_error<<"\n";
	      error = 0.0;
	      penalty = 0;
	  }

	  else if (counter == 1) {
	      std::cout<<"Knob No:"<<knob_no<<"\n";
	      switch(knob_no){
		  case 1: Kp += delKp;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Increasing Kp by"<<delKp<<"\n";
			  break;
		  case 2: Kp -= 2*delKp;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Decreasing Kp by 2 * "<<delKp<<"\n";
			  break;
	          case 3: Ki += delKi;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Increasing Ki by"<<delKi<<"\n";
			  break;
	          case 4: Ki -= 2*delKi;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Decreasing Ki by 2 * "<<delKi<<"\n";
			  break;
	          case 5: Kd += delKd;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Increasing Kd by"<<delKd<<"\n";
			  break;
		  case 6: Kd -= 2*delKd;
	                  pid.Init(Kp, Ki, Kd);
			  std::cout<<"Decreasing Kd by 2 * "<<delKd<<"\n";
			  break;
	      }
	  }
          
	  if ((fabs(cte) > 5.0) && (counter > 1)){
              penalty += 50000;
	      std::cout<<" Out of Lane! ";
	      counter = no_steps;
          }

	  if ((counter<=0) || (counter>1)) {
	   error += (cte + penalty)*(cte + penalty);
	   //std::cout<<"cte / twderr: "<<cte<<" / "<<error<<" ";
	   pid.UpdateError(cte);
           steer_value = pid.TotalError(); 
	  }

	  if (counter == no_steps) {
	      error /= no_steps;
	      std::cout<<counter<<" Kp= "<<Kp<<" Ki= "<<Ki<<" Kd= "<<Kd;
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
	  std::cout<<" Avg. Error: "<<error<<" Best. Error: "<<best_error<<"\n";
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
	  if (((fabs(cte) > 5.0) && (counter > 1)) || (counter == 0)) {
	      msg = "42[\"reset\",{}]";
	      counter = 0;
	      penalty = 0;
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
    //std::cout << "Connected!!!" << std::endl;
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
