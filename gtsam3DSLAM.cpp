#include <iostream>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Quaternion.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/geometry/CameraSet.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <Eigen/Dense>
#include <fstream>
#include <vector>

struct Pose{
    int idx;
    double x; double y; double z; double qx; double qy; double qz; double qw;
};

struct Edge{
    int idx1; int idx2; double del_x; double del_y; double del_z; double del_qx; double del_qy; double del_qz;
     double del_qw;
    Eigen::MatrixXd mat0;
};

void writeDataToFile(gtsam::Values result, std::string filename);
void writeDataToFile(std::vector<Pose> pose_data, std::string filename);

int main(){
    std::ifstream myfile ("/home/bandunis/ROB530/HW3/parking-garage.g2o");
    std::string string1;
    std::vector<Pose> pose;    // Define variables to store data
    std::vector<Edge> edge;
    //Read the file
    while (myfile.good()){   
        myfile>>string1;
        if (string1=="VERTEX_SE3:QUAT"){
            int idx;
            double x;
            double y;
            double z;
            double qx;
            double qy;
            double qz;
            double qw;
            Pose p;
            myfile>>idx>>x>>y>>z>>qx>>qy>>qz>>qw;
            p.idx=idx;
            p.x=x;
            p.y=y;
            p.z=z;
            p.qx=qx;
            p.qy=qy;
            p.qz=qz;
            p.qw=qw;
            pose.push_back(p);
        };
        if (string1=="EDGE_SE3:QUAT") {
            int idx1; int idx2; double del_x; double del_y; double del_z; double del_qx;double del_qy;double del_qz;double del_qw;
            //Matrix entries
            double q11; double q12; double q13; double q14; double q15; double q16; double q22; double q23; double q24;
            double q25; double q26; double q33; double q34; double q35; double q36; double q44; double q45; double q46;
            double q55; double q56; double q66;   
            myfile>>idx1>>idx2>>del_x>>del_y>>del_z>>del_qx>>del_qy>>del_qz>>del_qw>>
            q11>>q12>>q13>>q14>>q15>>q16>>q22>>q23>>q24>>q25>>q26>>q33>>q34>>q35>>q36>>
            q44>>q45>>q46>>q55>>q56>>q66;

            Eigen::MatrixXd mat0(6, 6);
            mat0 << q11, q12, q13, q14, q15, q16, q12, q22, q23, q24, q25, q26, q13, q23, q33,
            q34, q35, q36, q14, q24, q34, q44, q45, q46, q15, q25, q35, q45, q55, q56, 
            q16, q26, q36, q46, q56, q66;
            Edge e;
            e.idx1=idx1; e.idx2=idx2; e.del_x=del_x; e.del_y=del_y; e.del_z=del_z; e.del_qx=del_qx;
            e.del_qy=del_qy; e.del_qz=del_qz; e.del_qw=del_qw; e.mat0=mat0;
            edge.push_back(e);
        };
    
    }
    std::pair<std::vector<Pose>,std::vector<Edge>> result2 (pose,edge);
    std::vector<Pose> pose_data;
    std::vector<Edge> edge_data;
    std::tie(pose_data,edge_data) = result2;
    writeDataToFile(pose_data,"original3D.txt");
    //Batch solution for 3D SLAM
    gtsam::NonlinearFactorGraph graph;
    //Adding prior
    gtsam::Quaternion qat_p(pose[0].qw, pose[0].qx, pose[0].qy, pose[0].qz);
    gtsam::Pose3 priorMean(qat_p, gtsam::Point3(pose[0].x, pose[0].y, pose[0].z));
    auto priorNoise = 
    gtsam::noiseModel::Gaussian::Information(edge[0].mat0);   
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(0, priorMean, priorNoise));
    
    //Add edge info i.e. Odometry
    for (int iter=0; iter<edge.size(); iter++){
        // std::cout<<edge[iter].idx1<<"  "<<edge[iter].idx2<<"\n";
        auto model =
        gtsam::noiseModel::Gaussian::Information(edge[iter].mat0);
        graph.add(gtsam::BetweenFactor<gtsam::Pose3>(edge[iter].idx1, edge[iter].idx2, 
        gtsam::Pose3(gtsam::Quaternion(edge[iter].del_qw, edge[iter].del_qx, edge[iter].del_qy, 
        edge[iter].del_qz), gtsam::Point3(edge[iter].del_x, edge[iter].del_y, edge[iter].del_z)
         ), model));
    };
    // Initializing the graph
    gtsam::Values initial;
    for (int iter=0; iter<pose.size(); iter++){
        initial.insert(pose[iter].idx, gtsam::Pose3(gtsam::Quaternion(pose[iter].qw, pose[iter].qx, 
        pose[iter].qy, pose[iter].qz), gtsam::Point3(pose[iter].x, 
        pose[iter].y, pose[iter].z)
         ));
    };

    //Batch Solution
    gtsam::Values result = gtsam::GaussNewtonOptimizer(graph, initial).optimize();
     writeDataToFile(result,"3DoptimizedNG_batchsol.txt");


    //Incremental GTSAM
    gtsam::ISAM2 isam=gtsam::ISAM2();   
    gtsam::Values result1;

    for (int it=0; it<pose.size(); iter++){
        gtsam::NonlinearFactorGraph graph2;
        gtsam::Values initialEstimate;
        int idx2=pose[it].idx;
        double x2=pose[it].x;
        double y2=pose[it].y;
        double z2=pose[it].z;
        double qx2=pose[it].qx;
        double qy2=pose[it].qy;
        double qz2=pose[it].qz;
        double qw2=pose[it].qw;

        if (idx2==0){
                auto priorNoise2 = 
                    gtsam::noiseModel::Gaussian::Information(edge[0].mat0); 
                graph2.add(gtsam::PriorFactor<gtsam::Pose3>(0, gtsam::Pose3(gtsam::Quaternion(qw2, qx2, qy2, qz2),
                gtsam::Point3(x2, y2, z2)), priorNoise2));
                initialEstimate.insert(idx2, gtsam::Pose3(gtsam::Quaternion(qw2, qx2, qy2, qz2),
                gtsam::Point3(x2, y2, z2)));
        }
        else{
            gtsam::Pose3 prevPose=result1.at<gtsam::Pose3>(idx2-1);
            initialEstimate.insert(idx2, prevPose);

            for (int j=0; j<edge.size(); j++){
                if (edge[j].idx2==idx2){
                        gtsam::noiseModel::Gaussian::shared_ptr model =
                            gtsam::noiseModel::Gaussian::Information(edge[j].mat0);
                        graph2.add(gtsam::BetweenFactor<gtsam::Pose3>(edge[j].idx1, 
                        edge[j].idx2, gtsam::Pose3(gtsam::Quaternion(edge[j].del_qw, 
                        edge[j].del_qx, edge[j].del_qy, edge[j].del_qz),gtsam::Point3(
                            edge[j].del_x, edge[j].del_y, edge[j].del_z)), model));
                }
            }
        }
        isam.update(graph2, initialEstimate);
        result1=isam.calculateEstimate();
    };
    writeDataToFile(result1,"3DoptimizedNG_incrsol.txt");
    return 0;
};

void writeDataToFile(gtsam::Values result, std::string filename){
    std::ofstream file(filename); 
    for(int i = 0; i < result.size(); i++){
        file<< result.at<gtsam::Pose3>(i).x() <<" " << result.at<gtsam::Pose3>(i).y() <<" " << result.at<gtsam::Pose3>(i).z()<<std::endl;
    }
    file.close();
    
}

void writeDataToFile(std::vector<Pose> pose_data, std::string filename){
    std::ofstream file(filename);
    for(int i = 0; i<pose_data.size();i++){
        file << pose_data[i].x << " " <<pose_data[i].y <<" "<<pose_data[i].z<<std::endl;
    }
    file.close();
}