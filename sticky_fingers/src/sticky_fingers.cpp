#include <gazebo/sensors/sensors.hh>
#include <gazebo_ros_control/gazebo_ros_control_plugin.h>
#include <ros/ros.h>
#include <sticky_fingers/StickyControl.h>

namespace gazebo{
	class StickyFingers : public gazebo::SensorPlugin{
		private:

			//State information
			bool sticky;
			physics::LinkPtr held_object;

			math::Pose ho_trans;

			double max_mass;

			sensors::ContactSensorPtr finger_sensor;
			physics::LinkPtr finger_link;
			physics::WorldPtr finger_world;

			event::ConnectionPtr updateConnection;

			//ROS communication
			ros::NodeHandle nh;
			ros::ServiceServer service;
			bool ControlCallback(
				sticky_fingers::StickyControlRequest& request,
				sticky_fingers::StickyControlResponse& response
			){
				ROS_INFO("CB.");
				if(this->sticky && !request.sticky_status){//We are sticky and should stop being such.
					this->sticky = false;//Stop being sticky.
					held_object = NULL;//Drop our held object (if any)
					this->finger_link->SetCollideMode("all");//Resume collisionality
					response.new_status = false;//Report what we just did.
					return true;
				}
				else if(!this->sticky && request.sticky_status){//We are not sticky and should be sticky...
					this->sticky = true;
					response.new_status = true;
					return true;
				}
				//Otherwise, we really don't have anything in particular to DO...
				response.new_status = sticky;
				return true;
			}

		public:
			void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf){

				this->sticky = false;
				this->held_object = NULL;

				this->max_mass = sdf->GetElement("capacity")->Get<double>();

				//Get things that require effort to look up and can be kept persistant.
				this->finger_sensor = boost::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
				this->finger_world = physics::get_world(finger_sensor->GetWorldName());
				this->finger_link = boost::dynamic_pointer_cast<physics::Link>(
					this->finger_world->GetEntity(finger_sensor->GetParentName())
				);

				//Set up ROS communication
				std::string fingername = finger_sensor->GetName();
				int a = 0;//No, it will NOT just accept an argument size of 0 without shenanigans. Annoying.
				ros::init(a, (char **) NULL, fingername+"_node");
				service = this->nh.advertiseService(
					"sticky_finger/" + fingername,
					&StickyFingers::ControlCallback,
					this
				);
				ROS_INFO(
					"Sticky finger node %s listening on topic [%s].",
					(fingername+"_node").c_str(),
					("sticky_finger/" + fingername).c_str()
				);

				//Activate the sensor.
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
					boost::bind(&StickyFingers::OnUpdate, this, _1)
				);
				this->finger_sensor->SetActive(true);
			}

			void OnUpdate(const common::UpdateInfo & info){
				if(this->sticky){//We don't really need to do anything at all unless we're in sticky mode...


					if(this->held_object == NULL){//Prospecting mode:

						msgs::Contacts allcon = this->finger_sensor->GetContacts();

						for(unsigned int i = 0; i < allcon.contact_size(); i++){

							//There seems to be no rhyme or reason as to which name Gazebo will register in the first slot.
							std::string cname;
							if(finger_sensor->GetCollisionName(0).compare(allcon.contact(i).collision1()) != 0){
								cname = allcon.contact(i).collision1();
							}
							else{
								cname = allcon.contact(i).collision2();
							}

							physics::LinkPtr candidate =
								boost::dynamic_pointer_cast<physics::Collision>(
									finger_world->GetEntity(cname)
								)
							->GetLink();

							if(!(candidate->GetModel()->IsStatic())){//Ignore static objects
								if(candidate->GetInertial()->GetMass() <= this->max_mass){//Ignore heavy objects
									//ROS_INFO("Grabbing link %s.", candidate->GetName().c_str());

									this->held_object = candidate;

									math::Pose finger_pose = finger_link->GetWorldCoGPose();
									math::Pose object_pose = held_object->GetWorldCoGPose();
									this->ho_trans = finger_pose.CoordPoseSolve(object_pose);

									this->finger_link->SetCollideMode("none");

									break;
								}
							}
						}
					}

					else{//Carrying mode
						held_object->SetWorldPose(
							ho_trans + finger_link->GetWorldCoGPose(),
							true,
							true
						);
						//CRUDE first-order approximation of velocity.
						//This is actually all that is required to prevent jitter.
						this->held_object->SetWorldTwist(
							finger_link->GetWorldLinearVel(),
							math::Vector3(0.0, 0.0, 0.0)
						);
					}
				}
			}
	};

	GZ_REGISTER_SENSOR_PLUGIN(StickyFingers)
}