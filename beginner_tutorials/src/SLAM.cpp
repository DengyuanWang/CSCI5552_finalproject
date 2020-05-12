#include "header.h"
#include <unordered_map>
#include <unordered_set>
#include "filter.cpp"

using namespace std;

class SLAM{
  public:
	beginner_tutorials::integrated_msg integrated_msg;
	Eigen::MatrixXd Sigma_n_glob;
	Eigen::Matrix2d Sigma_ms_glob;
        Eigen::VectorXd x_hat_t_glob;
        Eigen::VectorXd x_hat_tpdt_glob;
        Eigen::MatrixXd Sigma_x_t_glob;
        Eigen::MatrixXd Sigma_x_tpdt_glob;

        bool enable_manual_input;
        int count;

  public: 
	void init_SLAM();
	void do_SLAM();
	void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);
	void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::Matrix2d Sigma_ms,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);

    void EKFSLAMRelPosUpdate2(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t,  Eigen::Matrix2d Sigma_ms,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);
}; //std::vector<Eigen::MatrixXd> Sigma_ms,


void SLAM::init_SLAM(){
	Eigen::VectorXd x_T = Eigen::Vector3d(0,0,0);
     	Sigma_n_glob = 0.03*0.03 * Eigen::Matrix2d::Identity();
	Sigma_ms_glob = 0.05*0.05 * Eigen::Matrix2d::Identity();
        x_hat_t_glob = x_T;
        //x_hat_tpdt_glob = x_T;
        Sigma_x_t_glob = 0.05*0.05 * Eigen::Matrix3d::Identity();
        //Sigma_x_tpdt_glob;

	enable_manual_input = false;
	count = 0;
}


void SLAM::do_SLAM(){

	count = (count+1)%3;

	float a = integrated_msg.u_v,b = integrated_msg.u_w,dt = integrated_msg.delta_t;
	//cout<<"do slam start"<<"a is "<<a<<"b is"<<b<<"delta t is "<<dt<<endl;
	Eigen::VectorXd u = Eigen::Vector2d(a,b);
	//cout<<"do slam start"<<"a is "<<u[0]<<"b is"<<u[1]<<endl;
     	EKFSLAMPropagate(x_hat_t_glob,  Sigma_x_t_glob,  u,  Sigma_n_glob, dt ,
     		x_hat_tpdt_glob, Sigma_x_tpdt_glob);
     
     	x_hat_t_glob= x_hat_tpdt_glob;
     	Sigma_x_t_glob = Sigma_x_tpdt_glob;
	//cout<<"RelPose UPDATE start"<<endl;
	if(count==0){
	     	EKFSLAMRelPosUpdate(x_hat_t_glob, Sigma_x_t_glob, Sigma_ms_glob, x_hat_tpdt_glob, Sigma_x_tpdt_glob);


		//EKFSLAMRelPosUpdate2(x_hat_t_glob, Sigma_x_t_glob, Sigma_ms_glob, x_hat_tpdt_glob, Sigma_x_tpdt_glob);

		x_hat_t_glob = x_hat_tpdt_glob;
		Sigma_x_t_glob = Sigma_x_tpdt_glob;
	}
	//cout<<"RelPose UPDATE end"<<endl;
};

void SLAM::EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {       // u : velocity and angular velocity, Sigma_n: user-defined
    // TODO
    //cout<< "EKFSLAMPropagate start" << std::endl;
    if(enable_manual_input){
	u[0] = 0;
   	u[1] = 0;
	}
    
    Eigen::Matrix3d A_R;
    A_R << 1.0, 0.0, -dt * u[0] * std::sin(x_hat_t[2]),
        0.0, 1.0, dt* u[0] * std::cos(x_hat_t[2]),
        0.0, 0.0, 1.0;
    Eigen::MatrixXd N_R(3, 2);
    N_R << dt * std::cos(x_hat_t[2]), 0,
        dt* std::sin(x_hat_t[2]), 0,
        0, dt;
    //only update first three element in x_hat_tpdt
    x_hat_tpdt = x_hat_t;
    x_hat_tpdt[0] = x_hat_t[0] + dt * u[0] * std::cos(x_hat_t[2]);
    x_hat_tpdt[1] = x_hat_t[1] + dt * u[0] * std::sin(x_hat_t[2]);
    x_hat_tpdt[2] = x_hat_t[2] + dt * u[1];
    
    //cout<< "x propagate" << std::endl;

    int r = Sigma_x_t.rows();
    int c = Sigma_x_t.cols();

    Eigen::MatrixXd A;
    A.setIdentity(r, c);
    A.block<3, 3>(0, 0) = A_R;
    int r2 = Sigma_x_t.rows();
    int c2 = Sigma_n.cols();

    Eigen::MatrixXd N;
    N.setZero(r2, c2);
    N.block<3, 2>(0, 0) = N_R;
    
    //cout<< "x propagate2" << std::endl;

    //update Sigma_x_tpdt
    //copy value
    Sigma_x_tpdt = Sigma_x_t;
    //update first block
    Sigma_x_tpdt.block(0, 0, 3, 3) = A_R * Sigma_x_t.block(0, 0, 3, 3) * A_R.transpose() + N_R * Sigma_n * N_R.transpose();
    //update first row
    Sigma_x_tpdt.block(0, 3, 3, Sigma_x_tpdt.cols() - 3) = A_R * Sigma_x_t.block(0, 3, 3, Sigma_x_tpdt.cols() - 3);
    //update first col
    Sigma_x_tpdt.block(3, 0, Sigma_x_tpdt.rows() - 3, 3) = Sigma_x_t.block(3, 0, Sigma_x_tpdt.rows() - 3, 3) * A_R.transpose();
    //Sigma_x_tpdt = A*Sigma_x_t*A.transpose() + N*Sigma_n*N.transpose();
    //cout<< "EKFSLAMPropagate finished\n" << Sigma_x_tpdt.size() << std::endl;
    // Note that these we passed by reference, so to return, just set them

}


void SLAM::EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::Matrix2d Sigma_ms,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {  // Eigen::MatrixXd LaserRB , Sigma_ms MatrixXd Sigma_ms(2,2)
    // Pass in LaserScan measurement date
    // TODO
    // Laserscan measurements: distance + bearing in robot frame(LaserRB) >> x,y coordinates in robot frame(LaserXY)
    //val =  [----]
    //#include<math.h>
    // if isnan(val){
    //}
    //len = length(in.....range)
    //integrated_msg.layserScan.range[i] = nan/  0.011
    //ag = integrated_msg.layserScan.angle_min
    //ag += integrated_msg.layserScan.angle_increment

	int len = integrated_msg.pcloud.points.size();
        int num_meas = len;
	if(len<1){
	x_hat_tpdt = x_hat_t;
		Sigma_x_tpdt = Sigma_x_t;
		return;
	}
	Eigen::MatrixXd LaserXY(len,2);
	for (int i = 0; i < len; i++){
		LaserXY(i,0) = integrated_msg.pcloud.points[i].x;
		LaserXY(i,1) = integrated_msg.pcloud.points[i].y;
	}
    // LaserXY is a len X 2 matrix containing all the laserscan measurements
    //assign value into LaserXY here:
    if (enable_manual_input){
	    Eigen::MatrixXd LaserXY_(303,2);
	    for (int i = 0;i<101;i++){
		LaserXY_(i,0) = 1;
		LaserXY_(i,1) = i*0.01 -0.5;
	    }
	    for (int i = 101;i<202;i++){
		LaserXY_(i,0) = -1;
		LaserXY_(i,1) = (i - 101)*0.01 -0.5;
	    }
	    for (int i = 202;i<303;i++){
		LaserXY_(i,0) = (i - 202)*0.01 -0.5;
		LaserXY_(i,1) = 1;
	    }
	    LaserXY = LaserXY_;
	    num_meas = LaserXY_.rows();
	}
    
    

    //
    //cout<< "from robot frame to global frame" << std::endl;
 
    int sampleNum = 2;
    int iterNum = 50; // number of iterations
    double InlrRation = 0.5; // inlier ration
    int InlrThr = 50; //int(InlrRation*num_meas);  // inlire threshold, if NumInlr > threshold, a line detected 
    double DisThr = 0.02;  // distance threshold, if distance < DisThr, inlier detected
    
    
    int num_left = num_meas;  // number of points left in the set of measurements
    Eigen::MatrixXd newLaserXY = LaserXY; // measurement sets after removing the inliers on a line
    srand(time(0));
    int num_pts;
    int num_line = 0; // number of lines detected
    double dis;   // distance from point to the line
    //Eigen::MatrixXd LinePara; // line parameters
    vector<double> LineParax,LineParay,LineParaz;

    if(num_left<1){
	x_hat_tpdt = x_hat_t;
        Sigma_x_tpdt = Sigma_x_t;
        return;
	}
    //cout<< "before for loop" << std::endl;
    for (int i = 1; i <= iterNum; i++) {
	//cout<<"i is"<<i<<endl;
        if (num_left < InlrThr) {
		//cout<<"break"<<num_left<<"   "<<InlrThr<<endl;
            break;             // if the number of points left is smaller than the inliere threshold, exit the loop
        }
	//cout<<"assign vals 0-1 "<<num_left<<endl;
	//cout<<"break"<<"   "<<rand()<<"val = "<< rand() % num_left <<endl;
        int N1 = rand() % num_left;
        int N2 = N1;
	//cout<<"assign vals 0 "<<endl;
        while (N1 == N2) {
            N2 = rand() % num_left;
        }
	//cout<<"assign vals"<<endl;
        Eigen::Vector2d P1 = Eigen::Vector2d(newLaserXY(N1,0),
            newLaserXY(N1,1));
        Eigen::Vector2d P2 = Eigen::Vector2d(newLaserXY(N2,0),
            newLaserXY(N2,1));
	//cout<<"assign vals2"<<endl;
        Eigen::Vector2d P_12 = P1 - P2;
        Eigen::Vector2d P12 = P_12.normalized();
        Eigen::Vector2d normP =  Eigen::Vector2d(P12[1],
            -P12[0]);
        int count = 0;
        //Eigen::MatrixXd Inliers;
	unordered_set<int> used_idx;
	vector<double> in_x,in_y;
	//cout<<"inner for start"<<endl;
        for (int j = 0; j < num_left; j++) {
	    //cout<<"newLaserXY is "<<newLaserXY.rows()<<endl;
            dis = (newLaserXY(j,0) - P1[0] )* normP[0] + (newLaserXY(j,1) - P1[1] )* normP[1]; 
            dis = abs(dis);
            if (dis < DisThr) {  
		//cout<<"distance is "<<dis<<" j is "<<j<<endl;       
		//cout<<"here"<<newLaserXY.block(j,0,1,2)<<endl; 
	        used_idx.insert(j);      
                Eigen::MatrixXd Inliers_row = newLaserXY.block(j,0,1,2);
		//cout<<"Inliers_row is "<<Inliers_row<<endl;
                in_x.push_back(Inliers_row(0,0));
		in_y.push_back(Inliers_row(0,1));
                //Inliers.block(count,0,1,2) = Inliers_row;
                count = count + 1;
		
            }
        } // find all the points lying near the line
	//cout<<"inner for finished"<<"count is "<<count<<endl;
        if (count > InlrThr) { // line detected
            //least square
            
		Eigen::VectorXd in_X_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_x.data(), in_x.size());
		Eigen::VectorXd in_Y_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(in_y.data(), in_y.size());
		Eigen::MatrixXd Inliers(in_X_.rows(), in_X_.cols()+in_Y_.cols());
		Inliers<<in_X_,in_Y_;
	    //cout<<"assign val finished"<<endl;
            double sumX = 0.0;
            double sumY = 0.0;
            double sumXY = 0.0;
            double sumX2 = 0.0;
            int num_liers = Inliers.rows();
		Eigen::MatrixXd col1 = Inliers.block(0,0,Inliers.rows(),1);
		Eigen::MatrixXd col2 = Inliers.block(0,1,Inliers.rows(),1);
	    Eigen::VectorXd sum_xy = Inliers.colwise().sum();
	    sumX = sum_xy[0];sumY = sum_xy[1];
            Eigen::MatrixXd sum_XYm = (col1.cwiseProduct(col2)).rowwise().sum();
		sumXY=sum_XYm(0,0);
	    Eigen::MatrixXd sum_XXm = (col1.cwiseProduct(col1)).rowwise().sum();
	    	sumX2 = sum_XXm(0,0);
	/*
            for (int k = 0; k < Inliers.rows();k++) {
                sumX = sumX + Inliers(k,0);
                sumY = sumY + Inliers(k,1);
                sumXY = sumXY + Inliers(k,0) * Inliers(k,1);
                sumX2 = sumX2 + Inliers(k,0) * Inliers(k,0);                
            }
*/
            //cout<<"n times X squared"<<num_liers*sumX2<<endl;
            //cout<<"sum X times sum X"<<sumX*sumX<<endl;
            if (abs(num_liers*sumX2 - sumX*sumX) < 0.01) {
                //Eigen::MatrixXd LineParaRow;
                //LineParaRow << 1, 0, -sumX / num_liers;
		LineParax.push_back(1);LineParay.push_back(0);LineParaz.push_back(-sumX / num_liers);
                //LinePara.block(num_line, 0, 1, 3) = ;
		//cout<<"Line is vertical "<< endl;
                num_line = num_line + 1;
            } // line is vertical to X axis
            else {
                double a1 = (num_liers*sumXY - sumX*sumY) / (num_liers*sumX2 - sumX*sumX);
                double a0 = sumY / num_liers - a1 * sumX / num_liers;
                //Eigen::MatrixXd LineParaRow;
                //LineParaRow << a1, -1, a0;
		LineParax.push_back(a1);LineParay.push_back(-1);LineParaz.push_back(a0);
                //LinePara.block(num_line, 0, 1, 3) = LineParaRow;
                //cout<<"Line is not vertical "<< endl;
                num_line = num_line + 1;
            } // line parameters stored in LinePara
        
            // next delete all inliers from the newLaserXY
            
            int count_inliers = 0;
            //Eigen::MatrixXd newLaserXY1;
	    vector<double> newLaserXY1x,newLaserXY1y;
            num_left = newLaserXY.rows();
	    //cout<<"inner inner for start"<<endl;
            for (int m = 0; m < num_left;m++) { // pts in measurement;
                int flag = 1;
		if(used_idx.find(m)==used_idx.end()){
		//	cout<<"INTO FLAG 1 M IS "<<m<<newLaserXY.block(m, 0, 1, 2)<<endl;
                    Eigen::MatrixXd newLaserXY1_row=newLaserXY.block(m, 0, 1, 2);
			
		    newLaserXY1x.push_back(newLaserXY1_row(0,0));newLaserXY1y.push_back(newLaserXY1_row(0,1));
                    //newLaserXY1.block(count_inliers, 0, 1, 2) = newLaserXY1_row;          //
                    count_inliers = count_inliers + 1; 
		    
		}
            }
	    //cout<<"inner inner for end"<<endl;
	    if(newLaserXY1x.size()<1){
                num_left = 0;
	    	continue;
	    }
            Eigen::VectorXd newLaserXY1x_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(newLaserXY1x.data(), newLaserXY1x.size());
    
    	    Eigen::VectorXd newLaserXY1y_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(newLaserXY1y.data(), newLaserXY1y.size());
    	    Eigen::MatrixXd newLaserXY1(newLaserXY1x_.rows(), 2);            
	    newLaserXY1<<newLaserXY1x_,newLaserXY1y_;
            newLaserXY = newLaserXY1;
            num_left = newLaserXY.rows();
     	    //cout<<"inner inner assign val end"<<endl;
        } // LSQ method to fit a line based on all those inliers found(Inliers)     

    } // RANSAC LinePara
    //cout<<"num_line is "<<num_line<<endl;
    //cout<<"Ransac end"<<endl;
    // point landmark extraction
    if(num_line<1){
	x_hat_tpdt = x_hat_t;
    	Sigma_x_tpdt = Sigma_x_t;
        //cout<<"did you end here?"<<endl;
	return;
    }
    //cout<<"ASSIGN VAL AFTER RANSAC"<< LineParax.size()<<endl;
    Eigen::VectorXd LineParax_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LineParax.data(), LineParax.size());
    
    Eigen::VectorXd LineParay_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LineParay.data(), LineParay.size());
    
    Eigen::VectorXd LineParaz_ = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LineParaz.data(), LineParaz.size());
    Eigen::MatrixXd LinePara(LineParax_.rows(), 3);
    LinePara<<LineParax_,LineParay_,LineParaz_;
    
    //cout<<"ASSIGN VAL AFTER RANSAC FINISHED"<< LinePara<<endl;
    

    // compute landmark here; using LaserXY, the raw measurement;
    num_line = LinePara.rows();
    // cout << "the number of detected line is: " << num_line << endl;
    //Eigen::MatrixXd Landmark(num_line,2);
    vector<double> LandmarkX,LandmarkY;
    for (int pp = 0; pp < num_line;pp++) {
        double a = LinePara(pp,0);
        double b = LinePara(pp,1);
        double c = LinePara(pp,2);
        //cout<<"a is"<<a<<endl;
        //cout<<"b is"<<b<<endl;
        //cout<<"c is"<<c<<endl;
        for (int tt = 0; tt < LaserXY.rows(); tt++){
             double x_pc = LaserXY(tt,0); 
             double y_pc = LaserXY(tt,1);
             if (b == 0){  // perpendicular line
                double distance = abs(x_pc + c/a);
                if (distance < 0.06){ //identify x_pc,y_pc as inliers
                   LandmarkX.push_back(x_pc);LandmarkY.push_back(y_pc);
                }

             }

             else{ // not perpendicular line
                double distance = abs(a*x_pc + b*y_pc + c)/sqrt(a*a + b*b);
                if (distance < 0.06){ //identify x_pc,y_pc as inliers
                   LandmarkX.push_back(x_pc);LandmarkY.push_back(y_pc);
                }
             }
         }

     }
     Eigen::VectorXd Landmark_X = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LandmarkX.data(), LandmarkX.size());
    
     Eigen::VectorXd Landmark_Y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(LandmarkY.data(), LandmarkY.size()); 
     Eigen::MatrixXd Landmark(Landmark_X.rows(), 2);
     Landmark<<Landmark_X,Landmark_Y;
     
    







    double Mahalanobis_distance_Upthreshold = 5.9915 * 5.9915, Mahalanobis_distance_Lowthreshold = 0.10 * 0.10;
    int measure_count = 0;
    for (int ii = 0; ii < Landmark.rows(); ii++) {

        int measurementNumber = (x_hat_t.size() - 3) / 2;   // number of landmarks in the state

        ////cout<< "Here is Sigma_ms length:\n" << Sigma_ms.size() << std::endl;
        ////cout<< "Here is zs length:\n" << zs.size() << std::endl;

        double L_x = Landmark(ii,0);
        double L_y = Landmark(ii,1);
        double R_x = x_hat_t[0];
        double R_y = x_hat_t[1];
        double R_the = x_hat_t[2];


        std::pair<double, int> d_id(0, -1);
        Eigen::MatrixXd S_best, K_best, H_best;
        Eigen::VectorXd r_i_best;

        Eigen::VectorXd z(2,1);     // landmark measurement
        z[0] = std::cos(R_the)*(L_x - R_x) + std::sin(R_the)*(L_y - R_y);
        z[1] = -std::sin(R_the)*(L_x - R_x) + std::cos(R_the)*(L_y - R_y);
        //z[0] = sqrt((L_x - R_x) * (L_x - R_x) + (L_y - R_y) * (L_y - R_y));
        //z[1] = std::atan2(L_y - R_y, L_x - R_x) - R_the;

        for (int i3 = 1; i3 <= measurementNumber; i3++) {
            ////cout<< "Start check for measurement "<< i <<" in x_hat_t, total measurement is "<<measurementNumber<< std::endl;
            
            double Lxi = x_hat_t[3 + 2 * (i3 - 1)];
            double Lyi = x_hat_t[4 + 2 * (i3 - 1)];
            double r_hat = sqrt((Lxi - R_x)*(Lxi - R_x) + (Lyi - R_y)*(Lyi - R_y));
            Eigen::VectorXd r_i;   // innovation 2 X 1
            Eigen::MatrixXd HR(2, 3);
            HR << -std::cos(R_the), -std::sin(R_the),-std::sin(R_the)*(Lxi - R_x) + std::cos(R_the)*(Lyi - R_y),
                  std::sin(R_the), -std::cos(R_the),-std::cos(R_the)*(Lxi - R_x) - std::sin(R_the)*(Lyi - R_y);
            //HR << (R_x - Lxi) / r_hat, (R_y - Lyi) / r_hat, 0,
            //    (Lyi - R_y) / ((r_hat) * (r_hat)), (Lxi - R_x) / ((r_hat) * (r_hat)), -1;
            Eigen::MatrixXd HLi(2, 2);
            HLi << std::cos(R_the), std::sin(R_the),
                 -std::sin(R_the), std::cos(R_the);
            //HLi << -(R_x - Lxi) / r_hat, -(R_y - Lyi) / r_hat,
            //    -(Lyi - R_y) / ((r_hat) * (r_hat)), -(Lxi - R_x) / ((r_hat) * (r_hat));

            Eigen::MatrixXd H;
            H = Eigen::MatrixXd::Zero(2, 3 + 2 * measurementNumber);
            H.block<2, 3>(0, 0) = HR;
            H.block<2, 2>(0, 2 * i3 + 1) = HLi;

            ////cout<< "Calculate H finished:\n" << H.size() << std::endl;
            Eigen::MatrixXd S;
            S = H * Sigma_x_t * H.transpose() + Sigma_ms;  // S = H * Sigma_x_t * H.transpose() + Sigma_ms[measure_count]
            ////cout<< "Calculate S finished:\n" << S.size() << std::endl;
            Eigen::VectorXd h_xLi0(2); // estimated measurement
            h_xLi0[0] = std::cos(R_the)*(Lxi - R_x) + std::sin(R_the)*(Lyi - R_y);
            h_xLi0[1] = -std::sin(R_the)*(Lxi - R_x) + std::cos(R_the)*(Lyi - R_y);

            r_i = z - h_xLi0; // innovation

            Eigen::VectorXd dv = r_i.transpose() * S.inverse().eval() * r_i;
            double d = dv[0];
            ////cout<< "Current d is :\n" << d << std::endl;
            if (d_id.second == -1 || d_id.first > d) {
                d_id = std::make_pair(d, i3);

                H_best = H;
                S_best = S;
                r_i_best = r_i;
                ////cout<< "Set D_id:\n" << d_id.first<<" | "<< d_id.second << std::endl;
            }
            else {
                //do nothing
            }
        }
        if (d_id.second == -1 || d_id.first > Mahalanobis_distance_Upthreshold) {
            //meet a new landmark
            ////cout<< "Start inserting new landmark measure_count="<<measure_count << std::endl;
            // L_x,L_y are new landmark coordinates;
            
            
            

            double r_hat = sqrt((L_x - R_x) * (L_x - R_x) + (L_y - R_y) * (L_y - R_y));
            Eigen::MatrixXd HR(2, 3);
            //HR << -std::cos(R_the), -std::sin(R_the),-std::sin(R_the)*(0 - R_x) + std::cos(R_the)*(0 - R_y),
            //      std::sin(R_the), -std::cos(R_the),-std::cos(R_the)*(0 - R_x) - std::sin(R_the)*(0 - R_y);
            HR << -std::cos(R_the), -std::sin(R_the),-std::sin(R_the)*(L_x - R_x) + std::cos(R_the)*(L_y - R_y),
                  std::sin(R_the), -std::cos(R_the),-std::cos(R_the)*(L_x - R_x) - std::sin(R_the)*(L_y - R_y);
            Eigen::MatrixXd HLi(2, 2);
            HLi << std::cos(R_the), std::sin(R_the),
                 -std::sin(R_the), std::cos(R_the);

            //Eigen::VectorXd h_xLi0(2);
            //h_xLi0[0] = std::cos(x_hat_t[2]) * (0 - x_hat_t[0]) + std::sin(x_hat_t[2]) * (0 - x_hat_t[1]);
            //h_xLi0[1] = -std::sin(x_hat_t[2]) * (0 - x_hat_t[0]) + std::cos(x_hat_t[2]) * (0 - x_hat_t[1]);


            // update state
            Eigen::VectorXd new_val(2,1);
            new_val << L_x,
                L_y;
            //update
            //std:://cout<<"Update x_hat_t new_val is"<<new_val<<std::endl;

            Eigen::VectorXd new_x_hat_t(x_hat_t.size() + 2);
            new_x_hat_t << x_hat_t, 
                new_val;
            x_hat_t = new_x_hat_t;                                                  // dimension may not agree
            
            //std:://cout<<"Update x_hat_t finished"<<std::endl;
            int rows_n, cols_n;
            rows_n = Sigma_x_t.rows();
            cols_n = Sigma_x_t.cols();
            //std:://cout<<"row is:"<<rows_n<<" col is "<<cols_n<<std::endl;
            //new right column
            //std:://cout<<"new right column Start"<<std::endl;
            Eigen::MatrixXd HR_T__HLkp1_neginvT;
            HR_T__HLkp1_neginvT = HR.transpose() * HLi.transpose().inverse().eval();

            Eigen::MatrixXd right_col;
            right_col = -Sigma_x_t.block(0, 0, rows_n, 3) * HR_T__HLkp1_neginvT;
            //std:://cout<<"new right column finished\n"<< right_col <<std::endl;
            //new bottom row
            Eigen::MatrixXd HLkp1_neginv__HR;
            HLkp1_neginv__HR = HLi.inverse().eval() * HR;
            Eigen::MatrixXd bot_row;
            bot_row = -HLkp1_neginv__HR * Sigma_x_t.block(0, 0, 3, cols_n);
            //std:://cout<<"new bot row finished\n"<< bot_row <<std::endl;

            //bot right block
            Eigen::MatrixXd last_block;
            last_block = HLi.inverse().eval() * (HR * Sigma_x_t.block(0, 0, 3, 3) * HR.transpose() + Sigma_ms) * HLi.transpose().inverse().eval();
            //std:://cout<<"new last block finished\n"<< last_block <<std::endl;

            Eigen::MatrixXd new_Sigma(rows_n + 2, cols_n + 2);

            //std:://cout<<"eql1\n"<<new_Sigma.block(0,0,rows_n,cols_n).size()<<std::endl;
            new_Sigma.block(0, 0, rows_n, cols_n) = Sigma_x_t;
            //std:://cout<<"eql2\n"<<new_Sigma.block(0,cols_n,rows_n,2).size()<<std::endl;
            new_Sigma.block(0, cols_n, rows_n, 2) = right_col;
            //std:://cout<<"eql3\n"<<new_Sigma.block(rows_n,0,2,cols_n).size()<<std::endl;
            new_Sigma.block(rows_n, 0, 2, cols_n) = bot_row;
            //std:://cout<<"eql4\n"<<new_Sigma.block(rows_n,cols_n,2,2).size()<<std::endl;
            new_Sigma.block(rows_n, cols_n, 2, 2) = last_block;

            Sigma_x_t = new_Sigma;

            ////cout<< "inserting new landmark measure_count="<<measure_count<<" finished"<< std::endl;
        }
        else if (d_id.first < Mahalanobis_distance_Lowthreshold) {
            //already met this landmark before
            ////cout<< "Start updating existed landmark measure_count="<< measure_count << std::endl;
            ////cout<< "Sigma_x_t:["<< Sigma_x_t.rows()<<" "<<Sigma_x_t.cols()<<"]" << std::endl;
            ////cout<< "H_best:["<< H_best.rows()<<" "<<H_best.cols()<<"]" << std::endl;
            ////cout<< "S_best:["<< S_best.rows()<<" "<<S_best.cols()<<"]" << std::endl;

            K_best = Sigma_x_t * H_best.transpose() * S_best.inverse().eval();

            Eigen::MatrixXd I_KH;
            I_KH = K_best * H_best;
            I_KH = Eigen::MatrixXd::Identity(I_KH.rows(), I_KH.cols()) - I_KH;
            ////cout<< "Here is the matrix K:\n" << K << std::endl;
            // Note that these we passed by reference, so to return, just set them
            ////cout<< "Start updating x_hat_t" << std::endl;
            x_hat_t = x_hat_t + K_best * r_i_best;
            ////cout<< "Here is the matrix x_hat_tpdt:\n" << x_hat_tpdt << std::endl;
            ////cout<< "Start updating Sigma_x_t" << std::endl;
            Sigma_x_t = I_KH * Sigma_x_t * I_KH.transpose() + K_best * Sigma_ms * K_best.transpose();
            ////cout<< "Existed landmark updated measure_count="<<measure_count<< std::endl;
        }



        measure_count = measure_count + 1;
    }
    // For each measurement, check if it matches any already in the state, and run an update for it.

    // For every unmatched measurement make sure it's sufficiently novel, then add to the state.

    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
    //cout << "number of landmark in state is" << (x_hat_t.rows() - 3)/2 <<endl;
    //cout << "X_coordinate is " << x_hat_t[3] << " Y_coordinate is " << x_hat_t[4] << std::endl;
    //cout << "X_coordinate is " << x_hat_t[5] << " Y_coordinate is " << x_hat_t[6] << std::endl;
    //cout << "X_coordinate is " << x_hat_t[7] << " Y_coordinate is " << x_hat_t[8] << std::endl;
}



void SLAM::EKFSLAMRelPosUpdate2(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::Matrix2d Sigma_m_,
                         Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {
    // TODO
 	
    std::vector<Eigen::VectorXd> zs;
    std::vector<Eigen::MatrixXd> Sigma_ms;

    int len = integrated_msg.pcloud.points.size();
	cout<<"len is"<<len<<endl;
	
    if(len<1){
   	x_hat_tpdt = x_hat_t;
    	Sigma_x_tpdt = Sigma_x_t;
	return;
	}

    for (int i = 0; i < integrated_msg.pcloud.points.size(); i++) {
        float x = integrated_msg.pcloud.points[i].x;  // X coordinates in global frame
        float y = integrated_msg.pcloud.points[i].y;  // Y coordinates in global frame
	//cout<<"cloud is "<<x<<"  "<<y<<endl;

        Eigen::VectorXd z = Eigen::Vector2d(x,y);
	zs.push_back(z);
        Sigma_ms.push_back(Sigma_m_);
    }


    cout<<"finish creating zs and sigmams"<<endl;



    double Mahalanobis_distance_Upthreshold = 5.9915,Mahalanobis_distance_Lowthreshold=0.5;

    int measure_count = 0;
    for(int idx = 0;idx<zs.size();idx++){
        Eigen::VectorXd z = zs[idx];
        int measurementNumber = (x_hat_t.size()-3)/2;
        
        //std::cout << "Here is Sigma_ms length:\n" << Sigma_ms.size() << std::endl;
        //std::cout << "Here is zs length:\n" << zs.size() << std::endl;
        
        std::pair<double,int> d_id(0,-1);
        Eigen::MatrixXd S_best,K_best,H_best;
        Eigen::VectorXd r_i_best;
        for(int i=1;i<=measurementNumber;i++){
            //std::cout << "Start check for measurement "<< i <<" in x_hat_t, total measurement is "<<measurementNumber<< std::endl;
            //if(sqrt(pow()+pow())>1)
            Eigen::VectorXd r_i;
            Eigen::MatrixXd HR(2,3);
            HR << -std::cos(x_hat_t[2]), -std::sin(x_hat_t[2]), -std::sin(x_hat_t[2])*(x_hat_t[2*i+1]-x_hat_t[0])
                                                                +std::cos(x_hat_t[2])*(x_hat_t[2*i+2]-x_hat_t[1]),
                std::sin(x_hat_t[2]), -std::cos(x_hat_t[2]), -std::cos(x_hat_t[2])*(x_hat_t[2*i+1]-x_hat_t[0])
                                                                -std::sin(x_hat_t[2])*(x_hat_t[2*i+2]-x_hat_t[1]);
            Eigen::MatrixXd HLi(2,2);
            HLi << std::cos(x_hat_t[2]), std::sin(x_hat_t[2]),
                -std::sin(x_hat_t[2]), std::cos(x_hat_t[2]);
            
            Eigen::MatrixXd H;
            H = Eigen::MatrixXd::Zero(2,3+2*measurementNumber);
            H.block<2,3>(0,0) = HR;
            H.block<2,2>(0,2*i+1) = HLi;
            
            //std::cout << "Calculate H finished:\n" << H.size() << std::endl;
            Eigen::MatrixXd S;
            S = H*Sigma_x_t*H.transpose() + Sigma_ms[measure_count];
            //std::cout << "Calculate S finished:\n" << S.size() << std::endl;
            Eigen::VectorXd h_xLi0(2);
            h_xLi0[0] = std::cos(x_hat_t[2])*(x_hat_t[2*i+1]-x_hat_t[0])+std::sin(x_hat_t[2])*(x_hat_t[2*i+2]-x_hat_t[1]);
            h_xLi0[1] = -std::sin(x_hat_t[2])*(x_hat_t[2*i+1]-x_hat_t[0])+std::cos(x_hat_t[2])*(x_hat_t[2*i+2]-x_hat_t[1]);
            
            r_i = z - h_xLi0;
            
            //Eigen::VectorXd dv = r_i.transpose()*S.inverse().eval()*r_i;
		Eigen::VectorXd dv = r_i.transpose()*r_i;
            double d = dv[0];
            //std::cout << "Current d is :\n" << d << std::endl;
            if(d_id.second==-1||d_id.first>d){
                d_id = std::make_pair(d,i);
                
                H_best = H;
                S_best = S;
                r_i_best = r_i;
                //std::cout << "Set D_id:\n" << d_id.first<<" | "<< d_id.second << std::endl;
            }else{
                //do nothing
            }
        }
        if(d_id.second==-1||d_id.first>Mahalanobis_distance_Upthreshold){
            //meet a new landmark
            //std::cout << "Start inserting new landmark measure_count="<<measure_count << std::endl;
            Eigen::MatrixXd HLi(2,2);
            HLi << std::cos(x_hat_t[2]), std::sin(x_hat_t[2]),
            -std::sin(x_hat_t[2]), std::cos(x_hat_t[2]);
            //std::cout<<"Update x_hat_t new_val is---------------"<<std::endl;
            Eigen::VectorXd h_xLi0(2);
            h_xLi0[0] = std::cos(x_hat_t[2])*(0-x_hat_t[0])+std::sin(x_hat_t[2])*(0-x_hat_t[1]);
            h_xLi0[1] = -std::sin(x_hat_t[2])*(0-x_hat_t[0])+std::cos(x_hat_t[2])*(0-x_hat_t[1]);
            
            
            //std::cout<<"-------------------Update x_hat_t new_val is-"<<std::endl;
            Eigen::VectorXd new_val;
            new_val = HLi.inverse().eval()*(z - h_xLi0);
            //update
            //std::cout<<"Update x_hat_t new_val is"<<new_val<<std::endl;
            
            Eigen::VectorXd new_x_hat_t(x_hat_t.size()+2);
            new_x_hat_t << x_hat_t, new_val;
            x_hat_t = new_x_hat_t;
            Eigen::MatrixXd HR(2,3);
            HR << -std::cos(x_hat_t[2]), -std::sin(x_hat_t[2]), -std::sin(x_hat_t[2])*(0-x_hat_t[0])
                                                                +std::cos(x_hat_t[2])*(0-x_hat_t[1]),
                std::sin(x_hat_t[2]), -std::cos(x_hat_t[2]), -std::cos(x_hat_t[2])*(0-x_hat_t[0])
                                                                -std::sin(x_hat_t[2])*(0-x_hat_t[1]);
            //std::cout<<"Update x_hat_t finished"<<std::endl;
            int rows_n,cols_n;
            rows_n = Sigma_x_t.rows();
            cols_n = Sigma_x_t.cols();
            //std::cout<<"row is:"<<rows_n<<" col is "<<cols_n<<std::endl;
            //new right column
            //std::cout<<"new right column Start"<<std::endl;
            Eigen::MatrixXd HR_T__HLkp1_neginvT;
            HR_T__HLkp1_neginvT = HR.transpose()*HLi.transpose().inverse().eval();
            
            Eigen::MatrixXd right_col;
            right_col = -Sigma_x_t.block(0,0,rows_n,3)*HR_T__HLkp1_neginvT;
            //std::cout<<"new right column finished\n"<< right_col <<std::endl;
            //new bottom row
            Eigen::MatrixXd HLkp1_neginv__HR;
            HLkp1_neginv__HR = HLi.inverse().eval()*HR;
            Eigen::MatrixXd bot_row;
            bot_row = -HLkp1_neginv__HR*Sigma_x_t.block(0,0,3,cols_n);
            //std::cout<<"new bot row finished\n"<< bot_row <<std::endl;
            
            //bot right block
            Eigen::MatrixXd last_block;
            last_block = HLi.inverse().eval()*(HR*Sigma_x_t.block(0,0,3,3)*HR.transpose()+Sigma_ms[measure_count])*HLi.transpose().inverse().eval();
            //std::cout<<"new last block finished\n"<< last_block <<std::endl;
            
            Eigen::MatrixXd new_Sigma(rows_n+2,cols_n+2);
            
            //std::cout<<"eql1\n"<<new_Sigma.block(0,0,rows_n,cols_n).size()<<std::endl;
            new_Sigma.block(0,0,rows_n,cols_n) = Sigma_x_t;
            //std::cout<<"eql2\n"<<new_Sigma.block(0,cols_n,rows_n,2).size()<<std::endl;
            new_Sigma.block(0,cols_n,rows_n,2) = right_col;
            //std::cout<<"eql3\n"<<new_Sigma.block(rows_n,0,2,cols_n).size()<<std::endl;
            new_Sigma.block(rows_n,0,2,cols_n) = bot_row;
            //std::cout<<"eql4\n"<<new_Sigma.block(rows_n,cols_n,2,2).size()<<std::endl;
            new_Sigma.block(rows_n,cols_n,2,2) = last_block;
            
            Sigma_x_t = new_Sigma;
            
            //std::cout << "inserting new landmark measure_count="<<measure_count<<" finished"<< std::endl;
        }else if(d_id.first<Mahalanobis_distance_Lowthreshold){
            //already met this landmark before
            //std::cout << "Start updating existed landmark measure_count="<< measure_count << std::endl;
            //std::cout << "Sigma_x_t:["<< Sigma_x_t.rows()<<" "<<Sigma_x_t.cols()<<"]" << std::endl;
            //std::cout << "H_best:["<< H_best.rows()<<" "<<H_best.cols()<<"]" << std::endl;
            //std::cout << "S_best:["<< S_best.rows()<<" "<<S_best.cols()<<"]" << std::endl;
            
            K_best = Sigma_x_t*H_best.transpose()*S_best.inverse().eval();
            
            Eigen::MatrixXd I_KH;
            I_KH = K_best*H_best;
            I_KH = Eigen::MatrixXd::Identity(I_KH.rows(), I_KH.cols()) - I_KH;
            //std::cout << "Here is the matrix K:\n" << K << std::endl;
            // Note that these we passed by reference, so to return, just set them
            //std::cout << "Start updating x_hat_t" << std::endl;
            x_hat_t = x_hat_t+K_best*r_i_best;
            //std::cout << "Here is the matrix x_hat_tpdt:\n" << x_hat_tpdt << std::endl;
            //std::cout << "Start updating Sigma_x_t" << std::endl;
            Sigma_x_t = I_KH*Sigma_x_t*I_KH.transpose() + K_best*Sigma_ms[measure_count]*K_best.transpose();
            //std::cout << "Existed landmark updated measure_count="<<measure_count<< std::endl;
        }
        
        
        
        measure_count = measure_count+1;
    }
    // For each measurement, check if it matches any already in the state, and run an update for it.
    
    // For every unmatched measurement make sure it's sufficiently novel, then add to the state.
    
    // Note that these we passed by reference, so to return, just set them

    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
    cout << "X_coordinate is " << x_hat_t[3] << "Y_coordinate is " << x_hat_t[4] << std::endl;
}




