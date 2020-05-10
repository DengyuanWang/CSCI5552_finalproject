#include "header.h"
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
        
        

  public: 
	void init_SLAM();
	void do_SLAM();
	void EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);
	void EKFSLAMRelPosUpdate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::Matrix2d Sigma_ms,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt);
}; //std::vector<Eigen::MatrixXd> Sigma_ms,


void SLAM::init_SLAM(){
	Eigen::VectorXd x_T = Eigen::Vector3d(0,0,3.1415926);
     	Sigma_n_glob = 0.03*0.03 * Eigen::Matrix2d::Identity();
	Sigma_ms_glob = 0.05*0.05 * Eigen::Matrix2d::Identity();
        x_hat_t_glob = x_T;
        //x_hat_tpdt_glob = x_T;
        Sigma_x_t_glob = 0.05*0.05 * Eigen::Matrix3d::Identity();
        //Sigma_x_tpdt_glob;
}


void SLAM::do_SLAM(){
	
	
	float a = integrated_msg.u_v,b = integrated_msg.u_w,dt = integrated_msg.delta_t;
	cout<<"do slam start"<<"a is "<<a<<"b is"<<b<<"delta t is "<<dt<<endl;
	Eigen::VectorXd u = Eigen::Vector2d(a,b);
	cout<<"do slam start"<<"a is "<<u[0]<<"b is"<<u[1]<<endl;
     	EKFSLAMPropagate(x_hat_t_glob,  Sigma_x_t_glob,  u,  Sigma_n_glob, dt ,
     		x_hat_tpdt_glob, Sigma_x_tpdt_glob);
     
     	x_hat_t_glob= x_hat_tpdt_glob;
     	Sigma_x_t_glob = Sigma_x_tpdt_glob;
	cout<<"RelPose UPDATE start"<<endl;
     	EKFSLAMRelPosUpdate(x_hat_t_glob, Sigma_x_t_glob, Sigma_ms_glob,
    		x_hat_tpdt_glob, Sigma_x_tpdt_glob);

	x_hat_t_glob = x_hat_tpdt_glob;
	Sigma_x_t_glob = Sigma_x_tpdt_glob;
	cout<<"RelPose UPDATE end"<<endl;
};

void SLAM::EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {       // u : velocity and angular velocity, Sigma_n: user-defined
    // TODO
    std::cout << "EKFSLAMPropagate start" << std::endl;

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
    
    std::cout << "x propagate" << std::endl;

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
    
    std::cout << "x propagate2" << std::endl;

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
    //std::cout << "EKFSLAMPropagate finished\n" << Sigma_x_tpdt.size() << std::endl;
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
    int len = integrated_msg.layserScan.ranges.size();
    double Bearing = integrated_msg.layserScan.angle_min;
    double Del_B = integrated_msg.layserScan.angle_increment;
    Eigen::MatrixXd LaserRB;
    int countValid = 0;
    for (int i_l = 0; i_l < len; i_l++) {
        if (isnan(integrated_msg.layserScan.ranges[i_l])) {
            // do nothing
        }
        else {
	    float val = integrated_msg.layserScan.ranges[i_l];
            LaserRB(countValid,0) = val;
            LaserRB(countValid,1) = Bearing;
            countValid = countValid + 1;
        }
        Bearing = Bearing + Del_B;
     }
    std::cout << "layserscan date convertion" << std::endl;

    int num_meas = LaserRB.rows();  // number of points in one set of Laser measurement
    Eigen::MatrixXd LaserXY;
    for (int i = 0; i < num_meas; i++) {
        LaserXY(i,0) = LaserRB(i,0) * std::cos(LaserRB(i,1));  // X coordinates in robot frame
        LaserXY(i,1) = LaserRB(i,0) * std::sin(LaserRB(i,1));  // Y coordinates in robot frame
     }

    //convert LaserXY to global frame
    double XR = x_hat_t[0];
    double YR = x_hat_t[1];
    double TheR = x_hat_t[2];
    for (int i = 0; i < num_meas; i++) {
        LaserXY(i,0) = XR + std::cos(TheR) * LaserXY(i,0) - std::sin(TheR) * LaserXY(i,1);  // X coordinates in global frame
        LaserXY(i,1) = YR + std::sin(TheR) * LaserXY(i,1) + std::cos(TheR) * LaserXY(i,1);;  // Y coordinates in global frame
    }
    
    std::cout << "from robot frame to global frame" << std::endl;
 
    int sampleNum = 2;
    int iterNum = 300; // number of iterations
    double InlrRation = 0.1; // inlier ration
    int InlrThr = int(InlrRation*num_meas);  // inlire threshold, if NumInlr > threshold, a line detected 
    double DisThr = 0.1;  // distance threshold, if distance < DisThr, inlier detected

    int num_left = num_meas;  // number of points left in the set of measurements
    Eigen::MatrixXd newLaserXY = LaserXY; // measurement sets after removing the inliers on a line
    srand(time(0));
    int num_pts;
    int num_line = 0; // number of lines detected
    double dis;   // distance from point to the line
    Eigen::MatrixXd LinePara; // line parameters

    std::cout << "before for loop" << std::endl;
    if(num_left<1){
	x_hat_tpdt = x_hat_t;
        Sigma_x_tpdt = Sigma_x_t;
        return;
	}
    for (int i = 1; i <= iterNum; i++) {
	cout<<"i "<<i<<endl;
        if (num_left < InlrThr) {
		cout<<"break"<<num_left<<"   "<<InlrThr<<endl;
            break;             // if the number of points left is smaller than the inliere threshold, exit the loop
        }
	cout<<"assign vals 0-1 "<<num_left<<endl;
	cout<<"break"<<"   "<<rand()<<"val = "<< rand() % num_left <<endl;
        int N1 = rand() % num_left;
        int N2 = N1;
	cout<<"assign vals 0 "<<endl;
        while (N1 == N2) {
            N2 = rand() % num_left;
        }
	cout<<"assign vals"<<endl;
        Eigen::Vector2d P1 = Eigen::Vector2d(newLaserXY(N1,0),
            newLaserXY(N1,1));
        Eigen::Vector2d P2 = Eigen::Vector2d(newLaserXY(N2,0),
            newLaserXY(N2,1));
	cout<<"assign vals2"<<endl;
        Eigen::Vector2d P_12 = P1 - P2;
        Eigen::Vector2d P12 = P_12.normalized();
        Eigen::Vector2d normP =  Eigen::Vector2d(P12[1],
            -P12[0]);
        int count = 0;
        Eigen::MatrixXd Inliers;
	cout<<"inner for start"<<endl;
        for (int j = 0; j < num_left; j++) {
            dis = newLaserXY(j,0) * normP[0] + newLaserXY(j,0) * normP[0];
            dis = abs(dis);
            if (dis < DisThr) {                
                Eigen::MatrixXd Inliers_row = newLaserXY.block(j,0,1,2);
                Inliers.block(count,0,1,2) = Inliers_row;
                count = count + 1;
            }
        } // find all the points lying near the line
	cout<<"inner for finished"<<endl;
        if (count > InlrThr) { // line detected
            //least square
            
            double sumX = 0.0;
            double sumY = 0.0;
            double sumXY = 0.0;
            double sumX2 = 0.0;
            int num_liers = Inliers.rows();
            for (int k = 0; k < Inliers.rows();k++) {
                sumX = sumX + Inliers(k,0);
                sumY = sumY + Inliers(k,1);
                sumXY = sumXY + Inliers(k,0) * Inliers(k,1);
                sumX2 = sumX2 + Inliers(k,0) * Inliers(k,0);                
            }
            if (abs(num_liers * sumX - sumX2) < 0.01) {
                Eigen::MatrixXd LineParaRow;
                LineParaRow << 1, 0, -sumX / num_liers;
                LinePara.block(num_line, 0, 1, 3) = LineParaRow;
                num_line = num_line + 1;
            } // line is vertical to X axis
            else {
                double a1 = (num_liers*sumXY - sumX*sumY) / (num_liers*sumX2 - sumX*sumX);
                double a0 = sumY / num_liers - a1 * sumX / num_liers;
                Eigen::MatrixXd LineParaRow;
                LineParaRow << a1, -1, a0;
                LinePara.block(num_line, 0, 1, 3) = LineParaRow;
                num_line = num_line + 1;
            } // line parameters stored in LinePara
        
            // next delete all inliers from the newLaserXY
            int flag = 1;
            int count_inliers = 0;
            Eigen::MatrixXd newLaserXY1;
            num_left = newLaserXY.rows();
		cout<<"inner inner for start"<<endl;
            for (int m = 0; m < num_left;m++) {
                for (int mm = 0; mm < Inliers.rows(); mm++) {
                    if (newLaserXY(m,0) == Inliers(mm,0) && newLaserXY(m,1) == Inliers(mm,1)) {
                        flag = 0;
                        break;
                    }
                }
                if (flag == 1) {
                    Eigen::MatrixXd newLaserXY1_row;
                    newLaserXY1_row << newLaserXY(m,0), newLaserXY(m,1);
                    newLaserXY1.block(count_inliers, 0, 1, 2) = newLaserXY1_row;
                    count_inliers = count_inliers + 1;
                }
            }
		cout<<"inner inner for end"<<endl;


            newLaserXY = newLaserXY1;
            num_left = newLaserXY.rows();
        } // LSQ method to fit a line based on all those inliers found(Inliers)     

    } // RANSAC LinePara

    // point landmark extraction
    Eigen::MatrixXd Landmark;
    num_line = LinePara.rows();
    for (int pp = 0; pp < num_line;pp++) {
        double a = LinePara(pp,0);
        double b = LinePara(pp,1);
        double c = LinePara(pp,2);
        if (LinePara(pp,1) == 0) {
            Landmark(pp,0) = LinePara(pp,1);
            Landmark(pp,1) = 0;
        }
        else {
            Landmark(pp,0) = (-a * c) / (a * a + b * b);
            Landmark(pp,0) = (-b * c) / (a * b + b * b);
        }
    }

    


    double Mahalanobis_distance_Upthreshold = 5.9915, Mahalanobis_distance_Lowthreshold = 0.5;
    int measure_count = 0;
    for (int ii = 0; ii < Landmark.rows(); ii++) {

        int measurementNumber = (x_hat_t.size() - 3) / 2;   // number of landmarks in the state

        //std::cout << "Here is Sigma_ms length:\n" << Sigma_ms.size() << std::endl;
        //std::cout << "Here is zs length:\n" << zs.size() << std::endl;

        double L_x = Landmark(ii,0);
        double L_y = Landmark(ii,1);
        double R_x = x_hat_t[0];
        double R_y = x_hat_t[1];
        double R_the = x_hat_t[2];


        std::pair<double, int> d_id(0, -1);
        Eigen::MatrixXd S_best, K_best, H_best;
        Eigen::VectorXd r_i_best;

        Eigen::VectorXd z(2,1);     // landmark measurement
        z[0] = sqrt((L_x - R_x) * (L_x - R_x) + (L_y - R_y) * (L_y - R_y));
        z[1] = std::atan2(L_y - R_y, L_x - R_x) - R_the;

        for (int i3 = 1; i3 <= measurementNumber; i3++) {
            //std::cout << "Start check for measurement "<< i <<" in x_hat_t, total measurement is "<<measurementNumber<< std::endl;
            
            double Lxi = x_hat_t[3 + 2 * (i3 - 1)];
            double Lyi = x_hat_t[4 + 2 * (i3 - 1)];
            double r_hat = sqrt((Lxi - R_x)*(Lxi - R_x) + (Lyi - R_y)*(Lyi - R_y));
            Eigen::VectorXd r_i;   // innovation 2 X 1
            Eigen::MatrixXd HR(2, 3);
            HR << (R_x - Lxi) / r_hat, (R_y - Lyi) / r_hat, 0,
                (Lyi - R_y) / ((r_hat) * (r_hat)), (Lxi - R_x) / ((r_hat) * (r_hat)), -1;
            Eigen::MatrixXd HLi(2, 2);
            HLi << -(R_x - Lxi) / r_hat, -(R_y - Lyi) / r_hat,
                -(Lyi - R_y) / ((r_hat) * (r_hat)), -(Lxi - R_x) / ((r_hat) * (r_hat));

            Eigen::MatrixXd H;
            H = Eigen::MatrixXd::Zero(2, 3 + 2 * measurementNumber);
            H.block<2, 3>(0, 0) = HR;
            H.block<2, 2>(0, 2 * i3 + 1) = HLi;

            //std::cout << "Calculate H finished:\n" << H.size() << std::endl;
            Eigen::MatrixXd S;
            S = H * Sigma_x_t * H.transpose() + Sigma_ms;  // S = H * Sigma_x_t * H.transpose() + Sigma_ms[measure_count]
            //std::cout << "Calculate S finished:\n" << S.size() << std::endl;
            Eigen::VectorXd h_xLi0(2); // estimated measurement
            h_xLi0[0] = sqrt((Lxi - R_x) * (Lxi - R_x) + (Lyi - R_y) * (Lyi - R_y));
            h_xLi0[1] = std::atan2(Lyi - R_y, Lxi - R_x) - R_the;

            r_i = z - h_xLi0; // innovation

            Eigen::VectorXd dv = r_i.transpose() * S.inverse().eval() * r_i;
            double d = dv[0];
            //std::cout << "Current d is :\n" << d << std::endl;
            if (d_id.second == -1 || d_id.first > d) {
                d_id = std::make_pair(d, i3);

                H_best = H;
                S_best = S;
                r_i_best = r_i;
                //std::cout << "Set D_id:\n" << d_id.first<<" | "<< d_id.second << std::endl;
            }
            else {
                //do nothing
            }
        }
        if (d_id.second == -1 || d_id.first > Mahalanobis_distance_Upthreshold) {
            //meet a new landmark
            //std::cout << "Start inserting new landmark measure_count="<<measure_count << std::endl;
            // L_x,L_y are new landmark coordinates;
            
            
            

            double r_hat = sqrt((L_x - R_x) * (L_x - R_x) + (L_y - R_y) * (L_y - R_y));
            Eigen::MatrixXd HR(2, 3);
            HR << (R_x - L_x) / r_hat, (R_y - L_y) / r_hat, 0,
                (L_y - R_y) / ((r_hat) * (r_hat)), (L_x - R_x) / ((r_hat) * (r_hat)), -1;
            Eigen::MatrixXd HLi(2, 2);
            HLi << -(R_x - L_x) / r_hat, -(R_y - L_y) / r_hat,
                -(L_y - R_y) / ((r_hat) * (r_hat)), -(L_x - R_x) / ((r_hat) * (r_hat));

            //Eigen::VectorXd h_xLi0(2);
            //h_xLi0[0] = std::cos(x_hat_t[2]) * (0 - x_hat_t[0]) + std::sin(x_hat_t[2]) * (0 - x_hat_t[1]);
            //h_xLi0[1] = -std::sin(x_hat_t[2]) * (0 - x_hat_t[0]) + std::cos(x_hat_t[2]) * (0 - x_hat_t[1]);


            // update state
            Eigen::VectorXd new_val(2,1);
            new_val << L_x,
                L_y;
            //update
            //std::cout<<"Update x_hat_t new_val is"<<new_val<<std::endl;

            Eigen::VectorXd new_x_hat_t(x_hat_t.size() + 2);
            new_x_hat_t << x_hat_t, 
                new_val;
            x_hat_t = new_x_hat_t;                                                  // dimension may not agree
            
            //std::cout<<"Update x_hat_t finished"<<std::endl;
            int rows_n, cols_n;
            rows_n = Sigma_x_t.rows();
            cols_n = Sigma_x_t.cols();
            //std::cout<<"row is:"<<rows_n<<" col is "<<cols_n<<std::endl;
            //new right column
            //std::cout<<"new right column Start"<<std::endl;
            Eigen::MatrixXd HR_T__HLkp1_neginvT;
            HR_T__HLkp1_neginvT = HR.transpose() * HLi.transpose().inverse().eval();

            Eigen::MatrixXd right_col;
            right_col = -Sigma_x_t.block(0, 0, rows_n, 3) * HR_T__HLkp1_neginvT;
            //std::cout<<"new right column finished\n"<< right_col <<std::endl;
            //new bottom row
            Eigen::MatrixXd HLkp1_neginv__HR;
            HLkp1_neginv__HR = HLi.inverse().eval() * HR;
            Eigen::MatrixXd bot_row;
            bot_row = -HLkp1_neginv__HR * Sigma_x_t.block(0, 0, 3, cols_n);
            //std::cout<<"new bot row finished\n"<< bot_row <<std::endl;

            //bot right block
            Eigen::MatrixXd last_block;
            last_block = HLi.inverse().eval() * (HR * Sigma_x_t.block(0, 0, 3, 3) * HR.transpose() + Sigma_ms) * HLi.transpose().inverse().eval();
            //std::cout<<"new last block finished\n"<< last_block <<std::endl;

            Eigen::MatrixXd new_Sigma(rows_n + 2, cols_n + 2);

            //std::cout<<"eql1\n"<<new_Sigma.block(0,0,rows_n,cols_n).size()<<std::endl;
            new_Sigma.block(0, 0, rows_n, cols_n) = Sigma_x_t;
            //std::cout<<"eql2\n"<<new_Sigma.block(0,cols_n,rows_n,2).size()<<std::endl;
            new_Sigma.block(0, cols_n, rows_n, 2) = right_col;
            //std::cout<<"eql3\n"<<new_Sigma.block(rows_n,0,2,cols_n).size()<<std::endl;
            new_Sigma.block(rows_n, 0, 2, cols_n) = bot_row;
            //std::cout<<"eql4\n"<<new_Sigma.block(rows_n,cols_n,2,2).size()<<std::endl;
            new_Sigma.block(rows_n, cols_n, 2, 2) = last_block;

            Sigma_x_t = new_Sigma;

            //std::cout << "inserting new landmark measure_count="<<measure_count<<" finished"<< std::endl;
        }
        else if (d_id.first < Mahalanobis_distance_Lowthreshold) {
            //already met this landmark before
            //std::cout << "Start updating existed landmark measure_count="<< measure_count << std::endl;
            //std::cout << "Sigma_x_t:["<< Sigma_x_t.rows()<<" "<<Sigma_x_t.cols()<<"]" << std::endl;
            //std::cout << "H_best:["<< H_best.rows()<<" "<<H_best.cols()<<"]" << std::endl;
            //std::cout << "S_best:["<< S_best.rows()<<" "<<S_best.cols()<<"]" << std::endl;

            K_best = Sigma_x_t * H_best.transpose() * S_best.inverse().eval();

            Eigen::MatrixXd I_KH;
            I_KH = K_best * H_best;
            I_KH = Eigen::MatrixXd::Identity(I_KH.rows(), I_KH.cols()) - I_KH;
            //std::cout << "Here is the matrix K:\n" << K << std::endl;
            // Note that these we passed by reference, so to return, just set them
            //std::cout << "Start updating x_hat_t" << std::endl;
            x_hat_t = x_hat_t + K_best * r_i_best;
            //std::cout << "Here is the matrix x_hat_tpdt:\n" << x_hat_tpdt << std::endl;
            //std::cout << "Start updating Sigma_x_t" << std::endl;
            Sigma_x_t = I_KH * Sigma_x_t * I_KH.transpose() + K_best * Sigma_ms * K_best.transpose();
            //std::cout << "Existed landmark updated measure_count="<<measure_count<< std::endl;
        }



        measure_count = measure_count + 1;
    }
    // For each measurement, check if it matches any already in the state, and run an update for it.

    // For every unmatched measurement make sure it's sufficiently novel, then add to the state.

    // Note that these we passed by reference, so to return, just set them
    x_hat_tpdt = x_hat_t;
    Sigma_x_tpdt = Sigma_x_t;
}
