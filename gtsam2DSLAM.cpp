#include <string>
#include <fstream>
#include <sstream>
#include <utility>
#include <iostream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/ISAM2Result.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <Eigen/Dense>




struct Pose{
    int id;
    double x;
    double y;
    double theta;
    Pose(int id, double x,double y,double theta) : id(id), x(x), y(y), theta(theta) {}
};


struct Edge{
    int i;
    int j;
    double x;
    double y;
    double theta;
    Eigen::Matrix<double,3,3> cov;  
    
    Edge(int i, int j, double x,double y, double theta, double argv[]) : i(i), j(j), x(x), y(y), theta(theta) {
        cov(0,0) = argv[0];
        cov(0,1) = argv[1];
        cov(1,0) = argv[1];
        cov(0,2) = argv[2];
        cov(2,0) = argv[2];
        cov(1,1) = argv[3];
        cov(1,2) = argv[4];
        cov(2,1) = argv[4];
        cov(2,2) = argv[5];
        cov = cov.inverse().eval();
    }
};
gtsam::Values batchSolution(std::vector<Pose> pose_data, std::vector<Edge> edge_data);
gtsam::Values incrementalSolution (std::vector<Pose> pose_data, std::vector<Edge> edge_data);
std::pair<std::vector<Pose>,std::vector<Edge>> load_g2ofile (std::string path); 
void printStruct(std::vector<Pose> pose_data);
void printStruct(std::vector<Edge> edge_data);
void writeDataToFile(gtsam::Values result, std::string filename);
void writeDataToFile(std::vector<Pose> pose_data, std::string filename);



int main(){

    std::string filename = "/home/bandunis/ROB530/HW3/input_INTEL_g2o.g2o";
    std::vector<Pose> pose_data;
    std::vector<Edge> edge_data;

    std::tie(pose_data,edge_data) = load_g2ofile(filename);
    writeDataToFile(pose_data,"original.txt");

    //Batch Solution
    gtsam::Values optimizedNG_batch = batchSolution(pose_data, edge_data);
    std::cout<< optimizedNG_batch.at<gtsam::Pose2>(0).x() <<" " << optimizedNG_batch.at<gtsam::Pose2>(0).y() <<std::endl;

    //Incremental Solution
    gtsam::Values optimizedNG_incremental = incrementalSolution(pose_data,edge_data);    
    std::cout<<"COMPLETE"<<std::endl;
    return 0;
}



gtsam::Values batchSolution(std::vector<Pose> pose_data, std::vector<Edge> edge_data){
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    // Create initial guess
    for(int i = 0; i < pose_data.size();i++){      
        initial.insert(pose_data[i].id, gtsam::Pose2(pose_data[i].x, pose_data[i].y, pose_data[i].theta));
    }
    
    //Adding the prior pose
    gtsam::Pose2 priorMean(pose_data[0].x,pose_data[0].y,pose_data[0].theta);
    auto priorNoise = gtsam::noiseModel::Gaussian::Covariance(edge_data[0].cov);
    graph.add(gtsam::PriorFactor<gtsam::Pose2>(0,priorMean,priorNoise));

    //Initializing the factor graph 
    for(int i = 0; i < edge_data.size(); i++){
        gtsam::Pose2 odometry(edge_data[i].x,edge_data[i].y,edge_data[i].theta);
        auto covariance = gtsam::noiseModel::Gaussian::Covariance(edge_data[i].cov);
        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(edge_data[i].i, edge_data[i].j, odometry, covariance));
    }

    gtsam::Values optimizedNG = gtsam::GaussNewtonOptimizer(graph, initial).optimize();
    writeDataToFile(optimizedNG,"optimizedNG_batchsol.txt");
    return optimizedNG;

}



gtsam::Values incrementalSolution (std::vector<Pose> pose_data, std::vector<Edge> edge_data){
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    gtsam::ISAM2 isam; 
    gtsam::Values optimizedISAM;

    for(auto pose : pose_data){
        if (pose.id == 0) {
            gtsam::Pose2 priorMean(pose.x,pose.y,pose.theta);
            auto priorNoise = gtsam::noiseModel::Gaussian::Covariance(edge_data[0].cov);
            graph.add(gtsam::PriorFactor<gtsam::Pose2>(0,priorMean,priorNoise));
            initial.insert(pose.id,priorMean);
        } else {
            gtsam::Pose2 prevPose = optimizedISAM.at<gtsam::Pose2>(pose.id-1);
            initial.insert(pose.id,prevPose);
            for (auto edge : edge_data){
                if (pose.id == edge.j){
                    gtsam::Pose2 odom (edge.x,edge.y,edge.theta);
                    auto covariance = gtsam::noiseModel::Gaussian::Covariance(edge.cov);
                    graph.add(gtsam::BetweenFactor<gtsam::Pose2>(edge.i,edge.j,odom,covariance));
                }
            }
        }
        isam.update(graph,initial);
        optimizedISAM = isam.calculateEstimate();
        graph.resize(0);
        initial.clear();
    }
    writeDataToFile(optimizedISAM,"optimizedNG_incrementalsol.txt");
    return optimizedISAM;
}





std::pair<std::vector<Pose>,std::vector<Edge>> load_g2ofile (std::string path) {
    std::ifstream infile(path);
    
    if(!infile){
        std::cout<<"Couldn't read the file"<<std::endl;
    } else{
        std::cout<<"found the file"<<std::endl;
    }
    std::string line;
    std::vector<Pose> pose_data;
    std::vector<Edge> edge_data;

    int id,i,j;
    double x,y,theta;
    double cov_elements[6];

    while(infile.good()){
               
        infile>>line;
        if(line == "VERTEX_SE2"){
            // std::cout<<"Found Vertex"<<std::endl;
            infile >>id>>x>>y>>theta;
            pose_data.push_back(Pose(id,x,y,theta));
            // std::cout<<"Created Pose object"<<std::endl;

        } else if(line == "EDGE_SE2"){
            // std::cout << "Found Edge"<< std::endl;
            infile>>i>>j>>x>>y>>theta;
            for(int iter = 0; iter<6; iter++){
                infile>>cov_elements[iter];
            }
            edge_data.push_back(Edge(i,j,x,y,theta,cov_elements));
        }
    }
    std::pair<std::vector<Pose>,std::vector<Edge>> result (pose_data,edge_data);
    
    return result;   
}



void printStruct(std::vector<Pose> pose_data){ 
    for(int i = 0; i<pose_data.size();i++){
        
        std::cout<<"POSE "<<i<<":"<<std::endl;
        std::cout<<"id: "<<pose_data[i].id<<std::endl;
        std::cout.precision(7);
        std::cout<<"x: "<<pose_data[i].x<<std::endl;
        std::cout<<"y: "<<pose_data[i].y<<std::endl;
        std::cout<<"theta: "<<pose_data[i].theta<<std::endl;
        std::cout<<std::endl;
    }  
}

void printStruct(std::vector<Edge> edge_data){
    for(int i = 0; i<edge_data.size();i++){
        
        std::cout<<"Edge "<<i<<":"<<std::endl;
        std::cout<<"i: "<<edge_data[i].i<<std::endl;
        std::cout<<"j: "<<edge_data[i].j<<std::endl;
        std::cout.precision(7);
        std::cout<<"x: "<<edge_data[i].x<<std::endl;
        std::cout<<"y: "<<edge_data[i].y<<std::endl;
        std::cout<<"theta: "<<edge_data[i].theta<<std::endl;
        std::cout<<"Cov: "<<edge_data[i].cov<<std::endl;
        std::cout<<std::endl;
    }
}

void writeDataToFile(gtsam::Values result, std::string filename){
    std::ofstream file(filename); 
    for(int i = 0; i < result.size(); i++){
        file<< result.at<gtsam::Pose2>(i).x() <<" " << result.at<gtsam::Pose2>(i).y() <<std::endl;
    }
    file.close();
    
}

void writeDataToFile(std::vector<Pose> pose_data, std::string filename){
    std::ofstream file(filename);
    for(int i = 0; i<pose_data.size();i++){
        file << pose_data[i].x << " " <<pose_data[i].y <<std::endl;
    }
    file.close();
}