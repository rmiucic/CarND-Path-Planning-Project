#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }
  int lane = 1;  // 0-left lane, 1-center lane, 2-right lane
  enum States { L0, L1, L2, L0_L1, L1_L0, L2_L1, L1_L2};
  States state = L1;  // 0-left lane, 1-center lane, 2-right lane
  double ref_vel = 0.0; //mph
  long int time_step=0;
  std::cout << "time_step,RV_id,RV_s,RV_d,RV_speed,HV_s,HV_d,HV_x,HV_y,HV_yaw,HV_speed,ref_vel,RV_in_HV_lane,RV_in_L0_zone,RV_in_L1_zone,RV_in_L2_zone,lane,state,previous_path_x_size" << std::endl;

  h.onMessage([&time_step,&state, &lane,&ref_vel,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double HV_x = j[1]["x"];
          double HV_y = j[1]["y"];
          double HV_s = j[1]["s"];
          double HV_d = j[1]["d"];
          double HV_yaw = j[1]["yaw"];
          double HV_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;


          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
            int prev_size = previous_path_x.size();

            if(prev_size > 0)
            {
              HV_s = end_path_s;
            }

            bool RV_in_HV_lane = false;
            bool RV_in_L0_zone = false;
            bool RV_in_L1_zone = false;
            bool RV_in_L2_zone = false;
            double inlane_ahead_zone = 30.0;
            double ahead_zone = 35.0;
            double behind_zone = 10.0;
            double center_offset = 0.3; 
            time_step++;
            //Go through all RVs
            int ix;
            for(ix = 0; ix < sensor_fusion.size(); ix++)
            {
              float RV_d = sensor_fusion[ix][6];
              double RV_vx = sensor_fusion[ix][3];
              double RV_vy = sensor_fusion[ix][4];
              double RV_speed = sqrt(RV_vx*RV_vx+RV_vy*RV_vy);
              double RV_s = sensor_fusion[ix][5];
              double RV_id = sensor_fusion[ix][0];
              
              //if using previous points can project
              RV_s += ((double)prev_size*0.02*RV_speed);
              //RV is in my lane
              if(RV_d < (2+4*lane+2) && RV_d > (2+4*lane-2))
              {
                //check if the RV is infront and with in inlane_ahead_zone
                if((RV_s > HV_s) && ((RV_s-HV_s)<inlane_ahead_zone))
                {
                  RV_in_HV_lane=true;
                }
              }
              
              if(RV_d < (2+4*0+2) && RV_d > (2+4*0-2)) //RV is in L0
              {
                //check if the RV is infront and with in inlane_ahead_zone
                if( ((RV_s >= HV_s) && ((RV_s-HV_s)<ahead_zone)) ||
                    ((RV_s < HV_s) && ((HV_s-RV_s)<behind_zone)) )
                {
                  RV_in_L0_zone=true;
                }
              }
              else if(RV_d < (2+4*1+2) && RV_d > (2+4*1-2)) //RV is in L1
              {
                //check if the RV is infront and with in inlane_ahead_zone
                if( ((RV_s >= HV_s) && ((RV_s-HV_s)<ahead_zone)) ||
                    ((RV_s < HV_s) && ((HV_s-RV_s)<behind_zone)) )
                {
                  RV_in_L1_zone=true;
                }
              }
              else if(RV_d < (2+4*2+2) && RV_d > (2+4*2-2)) //RV is in L2
              {
                //check if the RV is infront and with in inlane_ahead_zone
                if( ((RV_s >= HV_s) && ((RV_s-HV_s)<ahead_zone)) ||
                    ((RV_s < HV_s) && ((HV_s-RV_s)<behind_zone)) )
                {
                  RV_in_L2_zone=true;
                }
              }
            }
          /* Finite State Machine */
            switch(state)
            {
              case L0:
                lane = 0;
                if(RV_in_HV_lane && (RV_in_L1_zone == false))
                {
                  state = L0_L1;
                  lane = 1;
                }
                break;
              case L0_L1:
                if(HV_d < (2+4*1+center_offset) && HV_d > (2+4*1-center_offset))
                {
                  state = L1;
                }
                break;
              case L1_L0:
                if(HV_d < (2+4*0+center_offset) && HV_d > (2+4*0-center_offset))
                {
                  state = L0;
                }
                break;
              case L1:
                lane = 1;
                if(RV_in_HV_lane && (RV_in_L0_zone == false))
                {
                  state = L1_L0;
                  lane = 0;
                }
                else if(RV_in_HV_lane && (RV_in_L2_zone == false))
                {
                  state = L1_L2;
                  lane = 2;
                }
                break;
              case L1_L2:
                if(HV_d < (2+4*2+center_offset) && HV_d > (2+4*2-center_offset))
                {
                  state = L2;
                }
                break;
              case L2_L1:
                if(HV_d < (2+4*1+center_offset) && HV_d > (2+4*1-center_offset))
                {
                  state = L1;
                }
                break;
              case L2:
                lane = 2;
                if(RV_in_HV_lane && (RV_in_L1_zone == false))
                {
                  state = L2_L1;
                  lane = 1;
                }
                break;
            }
            
            
            //std::cout << state << std::endl;
            
            if(RV_in_HV_lane)
            {
                ref_vel -= 0.224;
            }
            else if(ref_vel < 49.0)
            {
              ref_vel += 0.224;
            }
            //std::cout << ref_vel << std::endl; 
            //sparsely separated points by 30m
            vector<double> ptsx; 
            vector<double> ptsy;

            //reference x,y yaw rate
            double ref_x = HV_x; 
            double ref_y = HV_y; 
            double ref_yaw = deg2rad(HV_yaw); 

            if (prev_size < 2)
            {
              double prev_HV_x = HV_x - cos(HV_yaw);
              double prev_HV_y = HV_y - sin(HV_yaw);

              ptsx.push_back(prev_HV_x);
              ptsx.push_back(HV_x);

              ptsy.push_back(prev_HV_y);
              ptsy.push_back(HV_y);
            }
            else
            {
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size-2];
              double ref_y_prev = previous_path_y[prev_size-2];
              ref_yaw = atan2(ref_y-ref_y_prev, ref_x);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            vector<double> next_wp0 = getXY(HV_s + 30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp1 = getXY(HV_s + 60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
            vector<double> next_wp2 = getXY(HV_s + 90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            for(int i = 0; i < ptsx.size(); i++)
            {
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
            }
            //sreate a spline
            tk::spline s;

            /*for(int i = 0; i < ptsx.size(); i++)
            { 
              std::cout << ptsx[i] << "," << ptsy[i]<< "," ;
            }
            std::cout << std::endl;
            */
            
            //set points of the spline
            s.set_points(ptsx,ptsy);

            vector<double> next_x_vals;
            vector<double> next_y_vals;

            for(int i = 0; i < previous_path_x.size(); i++)
            {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);

            }

            //calculate how to breakup spline
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));

            double x_add_on = 0;

            //fill up the rest of the path planer
            for (int i = 1; i<= 50 - prev_size; i++)
            {
              double N =(target_dist/(.02*ref_vel/2.24));
              double x_point = x_add_on + (target_x)/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              //rotate back to global coordinate system
              x_point = (x_ref * cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            for(ix = 0; ix < sensor_fusion.size(); ix++)
            {
              float RV_d = sensor_fusion[ix][6];
              double RV_vx = sensor_fusion[ix][3];
              double RV_vy = sensor_fusion[ix][4];
              double RV_speed = sqrt(RV_vx*RV_vx+RV_vy*RV_vy);
              double RV_s = sensor_fusion[ix][5];
              double RV_id = sensor_fusion[ix][0];
              std::cout << time_step << "," << RV_id << "," << RV_s << "," << RV_d << "," << RV_speed << "," << HV_s << "," << HV_d << "," << HV_x << "," << HV_y << "," << HV_yaw << "," << HV_speed << "," << ref_vel << "," << RV_in_HV_lane << "," << RV_in_L0_zone << "," << RV_in_L1_zone << "," << RV_in_L2_zone << "," << lane << "," << state  << "," << prev_size <<std::endl;
            }

          /*initial simple lane following */
            //double dist_inc = 0.3;
            //for (int i = 0; i < 50; ++i) 
            //{
            //    double next_s = HV_s + dist_inc*(i+1);
            //    double next_d = 6;
            //    vector<double> XY = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            //
            //    next_x_vals.push_back(XY[0]);
            //    next_y_vals.push_back(XY[1]);
            //}
          
          /* rmiucic end */  
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
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