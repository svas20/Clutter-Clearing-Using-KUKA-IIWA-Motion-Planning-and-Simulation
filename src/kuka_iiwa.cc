#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/geometry/scene_graph.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <memory>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/context.h>
#include<iostream>
#include <drake/geometry/scene_graph_inspector.h>
#include <drake/geometry/query_object.h>
#include <drake/math/rigid_transform.h>
#include <drake/systems/framework/output_port.h>
#include <drake/systems/analysis/simulator.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <thread> 

namespace ob = ompl::base;
namespace og = ompl::geometric;


class pick_and_place{
	private:
		drake::geometry::SceneGraph<double>* scene_graph{};
		drake::multibody::MultibodyPlant<double>* plant{};
		std::shared_ptr<drake::geometry::Meshcat> meshcat = std::make_shared<drake::geometry::Meshcat>();
		
		std::vector<Eigen::VectorXd> trajectory;
		
		struct ycb{
			std::string name;
			std::string file;
			template<typename Archive>
			void Serialize(Archive* a){
				a->Visit(DRAKE_NVP(name));
				a->Visit(DRAKE_NVP(file));
			}
		};
		
	public:
		Eigen::VectorXd ompl_Eigen(const ompl::base::State *state, int size){
		
		Eigen::VectorXd state_vec = Eigen::VectorXd::Zero(size);
		const ob::RealVectorStateSpace::StateType *pose = state->as<ob::RealVectorStateSpace::StateType>();
		//std::cout<<state_vec<<std::endl;
		for (int i =0; i<size; i++){
			state_vec(i) =(*pose)[i];
		}
		//std::cout<<state_vec;
		return state_vec;
		}
		std::vector<double> eigen_ompl(Eigen::VectorXd e){
			std::vector<double> v;
			v.resize(e.size());
			Eigen::VectorXd::Map(v.data(), e.size())=e;
			//std::cout<<v.data()<<std::endl;
		return v;
			
		}
		void load(){
			drake::systems::DiagramBuilder<double> builder;
			std::tie(plant,scene_graph) = drake::multibody::AddMultibodyPlantSceneGraph(&builder,0.0);
			drake::multibody::Parser parser(plant, scene_graph);
			parser.SetAutoRenaming(true);
			std::vector<drake::multibody::ModelInstanceIndex> model_instance = parser.AddModelsFromUrl("file:///home/astrid/drake/example/Test/models/env.dmd.yaml");
			plant->Finalize();
			//std::cout << "Number of models loaded: " << model_instance.size() << std::endl;
			//std::cout << "Model Name: " << plant->GetModelInstanceName(model_instance[0]) << std::endl;
			//std::cout << "Model Name: " << plant->GetModelInstanceName(model_instance[1]) << std::endl;
			drake::geometry::MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph, meshcat);
			std::unique_ptr<drake::systems::Diagram<double>> diagram = builder.Build();
		
			const drake::geometry::SceneGraphInspector<double>& scene_insp = scene_graph->model_inspector();
			const std::vector<drake::geometry::FrameId> frameid = scene_insp.GetAllFrameIds();
			const std::vector<drake::geometry::GeometryId> geoid = scene_insp.GetAllGeometryIds();
			std::unique_ptr<drake::systems::Context<double>> context =  diagram -> CreateDefaultContext();
			const drake::systems::Context<double>& plant_context_const = plant ->  GetMyContextFromRoot(*context);
			std::unique_ptr<drake::systems::Simulator<double>> sim = std::make_unique<drake::systems::Simulator<double>>(*diagram);
			sim->Initialize();
			drake::systems::Context<double> *plant_context = &diagram -> GetMutableSubsystemContext(*plant, &sim->get_mutable_context());
			drake::systems::State<double>& state = plant_context->get_mutable_state();
			const drake::systems::Context<double>& scene_context = diagram -> GetSubsystemContext(*scene_graph, *context);
			auto& query = scene_graph->get_query_output_port().Eval<drake::geometry::QueryObject<double>>(scene_context);
			/*for (const auto& id : frameid) {
		    		std::cout<<scene_insp.GetName(id);
		    		std::cout<<std::endl;
			}
			for (const auto& id : geoid){
				std::string geometry_name = scene_insp.GetName(id);
				drake::geometry::FrameId frame_id = scene_insp.GetFrameId(id);
				std::string frame_name = scene_insp.GetName(frame_id);
				const drake::math::RigidTransform<double>& pose = query.GetPoseInWorld(id);
				std::cout << "Geometry: " << geometry_name << " (Frame: " << frame_name << ") -> Pose: " << pose << std::endl;
			}*/
		
			const int dof =plant -> num_positions();
			//std::cout << dof << std::endl;
			auto space = std::make_shared<ob::RealVectorStateSpace>(dof);
			ob::RealVectorBounds bounds(space -> getDimension()); 
			Eigen::VectorXd pos_upper_lim = plant-> GetPositionUpperLimits();
			Eigen::VectorXd pos_lower_lim = plant-> GetPositionLowerLimits();
			for (int i=0 ; i<dof; i++){
				bounds.setHigh(i, pos_upper_lim(i));
				bounds.setLow(i, pos_lower_lim(i));
			}
			space -> setBounds(bounds);
			//std::cout<<"space"<<space->getDimension()<<std::endl;
			og::SimpleSetup ss(space);
			auto si = ss.getSpaceInformation();
			ss.setStateValidityChecker([this, plant_context, &dof, &space, &model_instance](const ob::State *state){
				const Eigen::VectorXd conf = this->ompl_Eigen(state, dof);
				//std::cout<<"conf size"<<conf.size()<<std::endl;
				for (const auto& instance : model_instance){
					//std::cout<<"instance"<<instance<<std::endl;
					this->plant->SetPositions(plant_context, instance, conf);
				}
				auto& get_query = this->plant-> get_geometry_query_input_port().Eval<drake::geometry::QueryObject<double>>(*plant_context);
				std::vector<drake::geometry::SignedDistancePair<double>> dist = get_query. ComputeSignedDistancePairwiseClosestPoints(0.5);
				bool collision_free = true;
				for (const auto& dist : dist){
					if(dist.distance<0.0){
						collision_free = false;
					}
				}
				return collision_free;
			});
			
			Eigen::VectorXd start_conf = Eigen::VectorXd::Zero(dof);
			//std::cout << dof << std::endl;
			start_conf << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			Eigen::VectorXd goal_conf = Eigen::VectorXd::Zero(dof);
			goal_conf << 0.0, 0.5, 0.0, -2, 0.0, 0.5, 0.0;
			ob::ScopedState<ob::RealVectorStateSpace>  start(space);
			start = eigen_ompl(start_conf);
			//std::cout<<start<<std::endl;
			ob::ScopedState<ob::RealVectorStateSpace>  goal(space);
			goal = eigen_ompl(goal_conf);
			ss.setStartAndGoalStates(start, goal);
			ss.getSpaceInformation() -> setStateValidityCheckingResolution(0.01);
			ss.setPlanner(std::make_shared<og::RRTstar>(si));
			ss.setup();
			ss.print();
			
			if (ss.solve(15) == ob::PlannerStatus::EXACT_SOLUTION){
				ss.simplifySolution(10);
				og::PathGeometric& path = ss.getSolutionPath ();
				path.interpolate(50);
				auto& path_states = path.getStates();
				
				for(auto& path : path_states){
					//std::cout<<path<<std::endl;
					trajectory.push_back(ompl_Eigen(path, dof));
				}
				
				 
			}else {
				return ;
			}
			
			for (const auto& conf : trajectory){
				for (const auto& instance : model_instance){
					plant->SetPositions(plant_context, instance, conf);
				}
				//auto event_collection = sim->get_system().AllocateCompositeEventCollection();
				//const drake::systems::EventCollection<drake::systems::PublishEvent<double>>& publish_events =event_collection->get_publish_events();
				//sim->get_system().Publish(sim->get_context(),publish_events);
				sim->AdvanceTo(sim->get_context().get_time() + 0.05);
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
			}
			std::reverse(std::begin(trajectory), std::end(trajectory));
			//drake::systems::Simulator<double> simulator(*diagram ,std::move(context));
			//simulator.Initialize();
			//simulator.AdvanceTo(2.0);
		}
		
			/*void clutter(){
				for (int i ; i<10 ; i++){
					ycb.push_back({ "thing" + std::to_string(i), "package://manipulation/hydro/" + ycb[object_num] });
				}
			
		}*/
	
};
int main(int argc, char** argv){
	pick_and_place iiwa;
	iiwa.load();
	return 0;

}
