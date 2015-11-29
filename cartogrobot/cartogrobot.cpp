
/**
  The backend reads HTTP pilot commands from superstar, writes serial
  commands to the arduino, and sends sensor data back up.

  Dr. Orion Lawlor, lawlor@alaska.edu, 2015-03-21 (public domain)
*/
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <sstream>

#ifdef _WIN32
#include <stdint.h>  // for unit8-16 and whatnot
#endif

//String helper:
#include "../include/string_util.h"

// Config file/cli read/write:
#include "../include/ini.h"
#include "../include/robot_config.h"

// Arduino comms:
#include "../include/serial.h" // serial port access

// Network comms:
#include "../include/osl/time_function.h" // timing
#include "../include/osl/webservice.h" // web client
#include "../include/json.h" // JSON parsing ("super easy JSON" library)

// Script execution
#ifndef	_WIN32
#include <unistd.h> /* for fork(), execl() */
#endif

/* Do linking right here */
#include "../include/ini.cpp"
#include "../include/robot_config.cpp"
#include "../include/string_util.cpp"
#include "../include/serial.cpp"
#include "../include/osl/socket.cpp"
#include "../include/osl/webservice.cpp"
#include "../include/json.cpp"

#include "../include/location_binary.h" /* for computer vision marker I/O */
#include "../include/osl/vec4.h" /* for vec3, used for arithmetic */

#ifndef M_PI
# define M_PI 3.1415926535897
#endif

void moose_sleep_ms(int delay_ms)
{
#if defined(__unix__)||defined(__MACH__)
	timespec t1;
	t1.tv_sec=delay_ms/1000; // remaining seconds
	t1.tv_nsec=1000*1000*(delay_ms%1000); // nanoseconds
	nanosleep(&t1,NULL); // limit rate to 100Hz, to be kind to serial port and network
#else
	Sleep(delay_ms);
#endif
}

void clean_exit(const char *why) {
	fprintf(stderr,"Backend exiting: %s\n",why);
	exit(1);
}

/**
  Read commands from superstar, and send them to the robot.
*/
class slam_backend {
private:
	osl::url_parser parseURL;
	osl::http_connection superstar; // HTTP keepalive connection
	std::string superstar_send_get(const std::string &path); // HTTP request

	std::string robotName;

public:
	slam_backend(std::string superstarURL, std::string robotName_)
		:parseURL(superstarURL), superstar(parseURL.host,0,parseURL.port),
		robotName(robotName_)
	{
		//stop();
	}
	
	void read_config(std::string config,const json::Value& configs,const int counter);
	
	int config_counter;

	/** Update pilot commands from network */
	void do_network(void);
	void read_network(const std::string &read_json);
	std::string send_network(void);
};

/** Send this HTTP get request for this path, and return the resulting string. */
std::string slam_backend::superstar_send_get(const std::string &path)
{
	for (int run=0;run<5;run++) {
		try {
			superstar.send_get(path);
			return superstar.receive();
		} catch (skt_error &e) {
			std::cout<<"NETWORK ERROR! "<<e.what()<<std::endl;
			std::cout<<"Retrying connection to superstar "<<parseURL.host<<":"<<parseURL.port<<std::endl;
			// Reopen HTTP connection
			superstar.close(); // close old connection
			if (run>0) sleep(1);
			// make new connection (or die trying)
			superstar.connect(parseURL.port,60+30*run);
			std::cout<<"Reconnected to superstar!\n";
		}
	}
	clean_exit("NETWORK ERROR talking to superstar.  Do you have wireless?");
	return "network error";
}

void slam_backend::read_config(std::string config,const json::Value& configs,const int counter)
{
	std::string path = "/superstar/" + robotName + "/config?get";
	try
	{
		for(size_t ii=0;ii<configs.ToArray().size();++ii)
			config+=configs.ToArray()[ii].ToString()+"\n";

		if(config_counter!=counter)
		{
			config_counter=counter;
			//tabula_setup(config);
		}

	} catch (std::exception &e) {
		printf("Exception while sending netwdork JSON: %s\n",e.what());
		// stop();
	}

	std::cout<<"config:  \n"<<config<<std::endl;
}

/**
 Do our network roundtrip to superstar.
*/
void slam_backend::do_network()
{
	double start=time_in_seconds();

	std::cout << "\033[2J\033[1;1H"; // clear screen	
	std::cout<<"Robot name: "<<robotName<<"\n";

	std::string send_json=send_network();
	std::cout<<"Outgoing sensors: "<<send_json<<"\n";

	std::string read_path=robotName+"/sensors";
	//std::string send_path=robotName+"/pilot,"+robotName+"/config";
	//std::string request=send_path+"?set="+send_json+"&get="+read_path;
	std::string read_json=superstar_send_get("/superstar/"+read_path);	//+request);

	std::cout<<"Incoming pilot commands: "<<read_json<<"\n";
	//read_network(read_json);

	double elapsed=time_in_seconds()-start;
	double per=elapsed;
	std::cout<<"Superstar:	"<<std::setprecision(1)<<per*1.0e3<<" ms/request, "<<1.0/per<<" req/sec\n\n";
	std::cout.flush();
}

/** Read this pilot data from superstar, and store into ourselves */
void slam_backend::read_network(const std::string &read_json)
{
	/*try {
		json::Array return_json=json::Deserialize(read_json);

		if(return_json.size()!=2)
			throw std::runtime_error("Invalid json received (expected 2 arguments) - "+read_json);

		json::Value pilot=return_json[0];
		json::Object config=return_json[1];

		if(config["configs"].GetType()==json::ArrayVal&&config["configs"].GetType()==json::ArrayVal&&config["counter"].IsNumeric())
			read_config("",config["configs"].ToArray(),config["counter"].ToInt());

		// Pull registered commands from JSON
		for (unsigned int i=0;i< commands.size();i++) commands[i]->do_command(pilot);

#ifndef	_WIN32
		//  On UNIX systems, run shell scripts from script/ directory
		static std::string last_cmd_arg="";
		static bool startup=true;
		std::string cmd_arg="";
		if(pilot["cmd"].GetType()==json::ObjectVal)
		{
			std::string run=pilot["cmd"]["run"];
			std::string arg=pilot["cmd"]["arg"];

			if (run.size()>0 && run.find_first_of("./\\\"")==std::string::npos) { // looks clean
				cmd_arg=run+arg;
				if (last_cmd_arg!=cmd_arg && !startup) { // new script command: run it

					// std::string path="./"+run;
					printf("RUNNING SCRIPT: '%s' with arg '%s'\n",
						run.c_str(),arg.c_str());

					if (fork()==0) {
						if (chdir("script")!=0) {
							printf("SCRIPT chdir FAILED\n");
						}
						else {
							execl("logger","logger",run.c_str(),arg.c_str(),(char *)NULL);
							perror("SCRIPT EXECUTE FAILED\n");
						}
						exit(0);
					}
				}
			}
		}
		startup=false;
		last_cmd_arg=cmd_arg;

#endif

		if (sim) {
			double distance_per_power=0.02; // meters per timestep
			double wheelbase=0.3; // meters

			if(pilot["power"].GetType()==json::ObjectVal&&pilot["power"]["L"].IsNumeric()&&pilot["power"]["R"].IsNumeric())
			{
				double delL=pilot["power"]["L"];
				double delR=pilot["power"]["R"];

				location.move_wheels(
					delL*distance_per_power,
					delR*distance_per_power,
					wheelbase);
			}
		}

	} catch (std::exception &e) {

		printf("Exception while processing network JSON: %s\n",e.what());
		printf("   Network data: %ld bytes, '%s'\n", (long)read_json.size(),read_json.c_str());
		// stop();
	}*/
}

/** Get the sensor reports to send across the network */
std::string slam_backend::send_network(void)
{
	std::string send_json="";
	/*try
	{ // send all registered sensor values
		json::Value sensorsJ=json::Object();
		location.copy_to_json(sensorsJ);
		for (unsigned int i=0;i< sensors.size();i++) sensors[i]->read_sensor(sensorsJ);
		for (unsigned int i=0;i< commands.size();i++) commands[i]->do_sensor(sensorsJ);

		send_json = json::Serialize(sensorsJ);
		send_json=uri_encode(send_json);
	} catch (std::exception &e) {
		printf("Exception while preparing network JSON to send: %s\n",e.what());
		// stop();
	}*/

	return send_json;
}

slam_backend *backend=NULL; // the singleton robot

int main(int argc, char *argv[])
{
	try
	{
		// MacOS runs double-clicked programs from homedir,  
		//   so cd to directory where this program lives.
		std::string exe_name=argv[0];
		std::cout<<"Executable name: "<<exe_name<<"\n";
		std::string dir_name=exe_name;
		while (dir_name.length()>0) {
			char c=*dir_name.rbegin(); // last letter
			if (c=='/' || c=='\\') break;
			else dir_name=dir_name.substr(0,dir_name.length()-1);
		}
		std::cout<<"Directory name: "<<dir_name<<"\n";
		if (chdir(dir_name.c_str())) { /* ignore chdir errors */ }
		
		
		robot_config_t config;

		config.from_file("config.txt");
		config.from_cli(argc,argv);

		std::cout<<"Connecting to superstar at "<<config.get("superstar")<<std::endl;

		backend=new slam_backend(config.get("superstar"),config.get("robot"));
		
		// Manually configuring Arduino (not via web) is a bad idea...
		//backend->tabula_setup(config.get("sensors")+"\n"+config.get("motors"));

		// talk to robot via backend
		while(true)
		{
			backend->do_network();
			//backend->send_serial();
			moose_sleep_ms(to_int(config.get("delay_ms")));
			//backend->read_serial();
			
			/*if((backend->_timeout)!=0)
			{	
				std::cout<<"Disconnected"<<std::endl;
				backend->reconnect(config.get("sensors")+"\n"+config.get("motors"),Serial);
			}*/
		}
		
		if(config.get("marker")!="")
		{
			//backend->location.update_vision(config.get("marker").c_str());
		}
	}

	catch(std::exception& error)
	{
		std::cout<<"ERROR! "<<error.what()<<std::endl;
		return 1;
	}

	return 0;
}