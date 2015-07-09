/***
 * This is an example of using turtlesim with publisher-subscriber.
 * It needs the address of a .txt file with the recipe as parameter in order to do his job.
 */
 
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <stdlib.h>

#define PI 3.141592

std::vector<double> distance;
std::vector<double> angle;

turtlesim::PoseConstPtr g_pose;
turtlesim::Pose g_goal;

unsigned int cantGiros=0, i=0, a=0, d=0;


// lista de los estados posibles 
enum State
{
  FORWARD,
  STOP_FORWARD,
  TURN,
  STOP_TURN,
};


// crea dos estados globales: estado actual y estado anterior
State g_state = FORWARD;
State g_last_state = FORWARD;
bool g_first_goal_set = false;

// almacena en g_pose el valor actual de pose
void poseCallback(const turtlesim::PoseConstPtr& pose)
{
  g_pose = pose;
}

// comprueba si la tortuga ha llegado a la meta (BOOL)

bool hasReachedGoal()
{
  return fabsf(g_pose->x - g_goal.x) < 0.03 && fabsf(g_pose->y - g_goal.y) < 0.03 && fabsf(g_pose->theta - g_goal.theta) < 0.003;
}

// comprueba si la tortuga se ha detenido (BOOL)
bool hasStopped()
{
  return g_pose->angular_velocity < 0.00005 && g_pose->linear_velocity < 0.00005;
}

// escribe en pantalla los valores de meta x,y,theta
void printGoal()
{
  ROS_INFO("New goal [%f %f, %f]", g_goal.x, g_goal.y, g_goal.theta);
}

// graba los valores de linear y angular en el publisher 
void commandTurtle(ros::Publisher twist_pub, float linear, float angular)
{
  geometry_msgs::Twist twist;
  twist.linear.x = linear;
  twist.angular.z = angular;
  twist_pub.publish(twist);
}

// detiene el movimiento forward
void stopForward(ros::Publisher twist_pub)
{
	double temp;
  if (hasStopped()) // si la tortuga se ha detenido:
  {
    ROS_INFO("Reached goal");
    g_state = TURN;
    g_goal.x = g_pose->x;	// mantiene los valores de x e y
    g_goal.y = g_pose->y;
		if((g_pose->theta + angle[a]) >= 2*PI)  {
    	g_goal.theta =g_pose->theta + angle[a] - 2*PI;		// mantiene el valor de theta menor a 2*PI
		}
		else g_goal.theta =g_pose->theta + angle[a];
		a++;
    printGoal();
  }
  else	// sino, da la orden de detenerla
  {
    commandTurtle(twist_pub, 0, 0);
  }
}

// detiene el movimiento de giro
void stopTurn(ros::Publisher twist_pub)
{
  if (hasStopped()) // si el giro se ha detenido:
  {
    ROS_INFO("Reached goal");


    g_state = FORWARD;
    g_goal.x = distance[d]* cos(g_pose->theta) + g_pose->x;
    g_goal.y = distance[d]* sin(g_pose->theta) + g_pose->y;
    g_goal.theta = g_pose->theta;
		d++;
    printGoal();
  }
  else	// sino, da la orden de detener el giro:
  {
    commandTurtle(twist_pub, 0, 0);
  }
}


// orden de movimiento forward
void forward(ros::Publisher twist_pub)
{
  if (hasReachedGoal())	// si ha llegado a la meta, da la orden de detener el movimiento:
  {
    g_state = STOP_FORWARD;
    commandTurtle(twist_pub, 0, 0);
  }
  else	// sino, da la orden de continuar el movimiento:
  {
    commandTurtle(twist_pub, 1.0, 0.0);
  }
}

// orden de movimiento angular
void turn(ros::Publisher twist_pub)
{
  if (hasReachedGoal())	// si se ha llegado a la meta, se detiene el giro:
  {
    g_state = STOP_TURN;
    commandTurtle(twist_pub, 0, 0);
    i++;
    if(i==cantGiros) {
	ROS_INFO("DONE");
	 ros::shutdown();
    }
  }
  else	// sino, se envia la orden de continuar girando:
  {
    commandTurtle(twist_pub, 0.0, 0.4);
  }
}

// funcion donde se ejecuta la logica, se llama cada vez que se cumple el tiempo configurado en timer
void timerCallback(const ros::TimerEvent&, ros::Publisher twist_pub)
{
  if (!g_pose)	// si g_pose==0, termina la funcion
  {
    return;
  }

  if (!g_first_goal_set) // si no hubiese una meta, ejecuta:
  {
    g_first_goal_set = true;
    g_state = FORWARD;
    g_goal.x = distance[d]*cos(g_pose->theta) + g_pose->x;
    g_goal.y = distance[d]*sin(g_pose->theta) + g_pose->y;
    g_goal.theta = g_pose->theta;
		d++;
    printGoal();
  }

  if (g_state == FORWARD)	// si el estado es FORWARD, ejecuta:
  {
    forward(twist_pub);	// envia la orden de movimiento a la funcion forward
  }
  else if (g_state == STOP_FORWARD)	// si el estado es STOP_FORWARD, ejecuta:
  {
    stopForward(twist_pub); // envia la orden de detenerse a la funcion stop_forward
  }
  else if (g_state == TURN) // si el estado es TURN, ejecuta:
  {
    turn(twist_pub); // envia la orden de giro a la funcion turn
  }
  else if (g_state == STOP_TURN) // si el estado es STOP_TURN, ejecuta:
  {
    stopTurn(twist_pub); // envia la orden de detener giro a la funcion stop_turn
  }
}

// funcion principal
int main(int argc, char** argv)
{
  ros::init(argc, argv, "draw_square");	
  ros::NodeHandle nh;
  ros::Subscriber pose_sub = nh.subscribe("turtle1/pose", 1, poseCallback); // se suscribe a turtle1/pose
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1); // publica en turtle1/pose
	std::string nombre;
  std::string line;
  std::string word;
  std::string::size_type sz;
  double num=0.0;
  unsigned int numLin=0;
  if ( argc > 2 )
    std::cout<<"usage: "<< argv[0] <<" <filename>\n";
  else {
    if(argc == 2)
        nombre=argv[1];
    else
      nombre="example.txt";
      std::ifstream myfile( nombre.c_str());
      if (myfile)
        {
        while (getline( myfile, line ))
        {
            std::istringstream iss(line);
            while(iss >> word)
            {
                num=atof(word.c_str());
                if(numLin==0) {
                    distance.push_back(num);
                }
                if(numLin==1) {
                    angle.push_back(num);
                }
            }
            numLin++;
        }
        myfile.close();
        }
        else std::cout << "fooey\n";
				cantGiros=distance.size() + 1;
        std::cout << "\ndistance: ";
        for(unsigned int j=0; j<distance.size();j++) {
            std::cout << distance[j] << " ";
        }
        std::cout << "\nangle: ";
        for(unsigned int j=0; j<angle.size();j++) {
            std::cout << angle[j] << " ";
        }
        std::cout << "\n";
    }
  ros::Timer timer = nh.createTimer(ros::Duration(0.002), boost::bind(timerCallback, _1, twist_pub));
  ros::spin();
}
