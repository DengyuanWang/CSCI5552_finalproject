#include "header.h"
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
}


void SLAM::do_SLAM(){
	
	
	float a = integrated_msg.u_v,b = integrated_msg.u_w,dt = integrated_msg.delta_t;
	//cout<<"do slam start"<<"a is "<<a<<"b is"<<b<<"delta t is "<<dt<<endl;
	Eigen::VectorXd u = Eigen::Vector2d(a,b);
	//cout<<"do slam start"<<"a is "<<u[0]<<"b is"<<u[1]<<endl;
     	EKFSLAMPropagate(x_hat_t_glob,  Sigma_x_t_glob,  u,  Sigma_n_glob, dt ,
     		x_hat_tpdt_glob, Sigma_x_tpdt_glob);
     
     	x_hat_t_glob= x_hat_tpdt_glob;
     	Sigma_x_t_glob = Sigma_x_tpdt_glob;
	//cout<<"RelPose UPDATE start"<<endl;
	EKFSLAMRelPosUpdate2(x_hat_t_glob, Sigma_x_t_glob, Sigma_ms_glob, x_hat_tpdt_glob, Sigma_x_tpdt_glob);
	x_hat_t_glob = x_hat_tpdt_glob;
	Sigma_x_t_glob = Sigma_x_tpdt_glob;
	//cout<<"RelPose UPDATE end"<<endl;
};

void SLAM::EKFSLAMPropagate(Eigen::VectorXd x_hat_t, Eigen::MatrixXd Sigma_x_t, Eigen::VectorXd u, Eigen::MatrixXd Sigma_n, double dt,
    Eigen::VectorXd& x_hat_tpdt, Eigen::MatrixXd& Sigma_x_tpdt) {       // u : velocity and angular velocity, Sigma_n: user-defined
    // TODO
    //cout<< "EKFSLAMPropagate start" << std::endl;

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
    cout<< "EKFSLAMPropagate finished\n" << Sigma_x_tpdt.size() << std::endl;
    // Note that these we passed by reference, so to return, just set them

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
	cout<<"cloud is "<<x<<"  "<<y<<endl;
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
            
            Eigen::VectorXd dv = r_i.transpose()*S.inverse().eval()*r_i;
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
}




