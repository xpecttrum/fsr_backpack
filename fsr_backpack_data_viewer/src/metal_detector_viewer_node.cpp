/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Jose Prado on 2015
*********************************************************************/

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <boost/signals.hpp>
//#include <boost/bind.hpp>
//#include <boost/mem_fn.hpp>
#include <geometry_msgs/PointStamped.h>
#include <metal_detector_msgs/Coil.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud.h>

using namespace Eigen;
using namespace sensor_msgs;
using namespace message_filters;
using namespace metal_detector_msgs;

#define MY_RAW_ESCALA 1.0  //quanto menor, maior a sensibilidade
//1.0 Schiebel
//5000.0 Vallon

//para a deteccao das minas
#define BUFFER_SIZE 50

//*** GLOBAIS *****
// --- For rviz
ros::Publisher marker_pub;
visualization_msgs::Marker mapa_plot, marker, points, line_strip, line_list;
ros::Publisher pcl_raw_pub;
//GEO POINTS
ros::Publisher pontos_prado_pub;
int tank_mine_limiar, personal_mine_limiar;
bool flag_sweep = true;

//#define TARGET_FRAME "/minefield"
//#define TARGET_FRAME "/base_footprint"
#define TARGET_FRAME "/odom"


//*****************

class MetalDetector
{
public:
    MetalDetector() : tf_(),  target_frame_(TARGET_FRAME)
    {
        md_sub_.subscribe(n_, "coils", 10);
        tf_filter_ = new tf::MessageFilter<metal_detector_msgs::Coil>(md_sub_, tf_, target_frame_, 10);
        tf_filter_->registerCallback( boost::bind(&MetalDetector::msgCallback, this, _1) );
        pub_ = n_.advertise<geometry_msgs::PointStamped>("coil_position", 10);
        
        //inicializacao das variaveis
     	qtd_pontos = 0;
		last_X = 0;
		last_Y = 0;
		last_Z = 0;
		inicio = 1;

		for(int i=0;i<BUFFER_SIZE;i++){
			coil_buffer[0][i] = 0;
		}
		for(int i=0;i<BUFFER_SIZE;i++){
			coil_buffer[1][i] = 0;
		}
		for(int i=0;i<BUFFER_SIZE;i++){
			coil_buffer[2][i] = 0;
		}
		
		buffer_index[0]=0;
		buffer_index[1]=0;
		buffer_index[2]=0;

		ajuste[0]=0;
		ajuste[1]=0;
		ajuste[2]=0;
		flag_calculou_as_modas_uma_vez=false;
		
		moda_ou_zero[0]=0;
		moda_ou_zero[1]=0;
		moda_ou_zero[2]=0;
		
        calculou_moda[0] = false;
        calculou_moda[1] = false;
        calculou_moda[2] = false;

		number_of_heter=0;
		number_of_mines=0;
		flag_detected = false;
		flag2_detected = false;
		nao_detections=0;
		
   
    }
    
    private:
    //buffer coil
	int coil_buffer[3][BUFFER_SIZE]; //3 porque sao 3 coils
	int buffer_index[3];
	int number_of_heter;
	int number_of_mines;
	bool flag_detected;
	bool flag2_detected;
	int nao_detections;
    
    int current_coil;
	int moda_ou_zero[3];
	int coil_canal0[3] ;
	int coil_canal1[3] ;

    bool calculou_moda[3];

	int ajuste[3];
	bool flag_calculou_as_modas_uma_vez;
	
	std_msgs::ColorRGBA c2;

    message_filters::Subscriber<metal_detector_msgs::Coil> md_sub_;
    tf::TransformListener tf_;
    tf::MessageFilter<metal_detector_msgs::Coil> * tf_filter_;
    ros::NodeHandle n_;
    std::string target_frame_;
    ros::Publisher pub_;
    
	long int qtd_pontos;
	char filename[30];
	//----
	float last_X, last_Y, last_Z;
	int inicio;

	// - Rainbow Color Transformation ---
	typedef struct {
		double r;       // percent
		double g;       // percent
		double b;       // percent
	} rgb;

	typedef struct {
		double h;       // angle in degrees
		double s;       // percent
		double v;       // percent
	} hsv;

//METHODS

/***************************************************************
Function Name : mean_function
Purpose : to find mean
Input : array of elements,no of elements
Return Value : Mean
Return Type : Float
****************************************************************/
float mean_function(float array[],int n)
{
	int i;
	float sum=0;
	for(i=0;i<n;i++)
	sum=sum+array[i];
	return (sum/n);
}
 
/***************************************************************
Function Name : median_function
Purpose : to find median
Input : array of elements,no of elements
Return Value : Median
Return Type : Float
****************************************************************/
 
float median_function(float a[],int n)
{
	float temp;
	int i,j;
	for(i=0;i<n;i++)
		for(j=i+1;j<n;j++)
		{
			if(a[i]>a[j])
			{
				temp=a[j];
				a[j]=a[i];
				a[i]=temp;
			}
		}
	if(n%2==0)
	return (a[n/2]+a[n/2-1])/2;
	else
	return a[n/2];
}
 
float mode_function(float a[],int n)
{
	return (3*median_function(a,n)-2*mean_function(a,n));
}
//**************************************

sensor_msgs::PointCloud cloud_msg;
geometry_msgs::Point32 ponto;

void publish_pcl_raw(float X,float Y, float Z, ros::Publisher *pub_ptr)
{
	
	ponto.x = X;
	ponto.y = Y;
	ponto.z = Z;

	cloud_msg.points.push_back(ponto);
	
	cloud_msg.header.stamp = ros::Time::now();
	cloud_msg.header.frame_id = TARGET_FRAME;

	pub_ptr->publish(cloud_msg);
	
	//comentar linha abaixo se quiser acumular a cloud, problema é que gera um rosbag muito grande
	//cloud_msg.points.pop_back();
}
	
void publishRvizMarker(float X,float Y, float Z, std_msgs::ColorRGBA corPonto){

	geometry_msgs::Point p;

    points.header.frame_id = TARGET_FRAME;
    points.header.stamp = ros::Time::now();
    points.ns = "Interesting Points";
    points.action = visualization_msgs::Marker::ADD;
    points.id = 3;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1; //0.1
    // Points are white
    points.color.r = 1.0f;
    points.color.g = 1.0f;
    points.color.b = 1.0f;
    points.color.a = 1.0f;
	
	p.x = X;
	p.y = Y;
	p.z = Z;

	points.points.push_back(p);
    //points.colors.push_back(corPonto);

	// Publish the marker
    fprintf(stderr,"\npublish marker\n");
    marker_pub.publish(points);

}


void encherBuffer(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr){

    if(coil_ptr->header.frame_id == "metal_detector_middle_coil"){
           //fprintf(stderr,"use_middle_buffer");
           current_coil = 0;
    }
    if(coil_ptr->header.frame_id == "left_coil"){
           //fprintf(stderr,"use_left_buffer");
           current_coil = 1;
    }
    if(coil_ptr->header.frame_id == "right_coil"){
           //fprintf(stderr,"use_right_buffer");
           current_coil = 2;
    }

   //separa as coils e inverte o sinal - o detector da Schiebel só tem 1 canal, dupliquei para ambos os canais
   coil_canal0[current_coil] = coil_ptr->channel[0];
   coil_canal1[current_coil] = coil_ptr->channel[0];


   //buffer para calculo da moda, 3 buffers, um para cada coil
   if(buffer_index[current_coil] < (BUFFER_SIZE-1)){//encher o buffer
       coil_buffer[current_coil][buffer_index[current_coil]] = coil_canal1[current_coil];
       buffer_index[current_coil]++;
   }


   //manter buffer circular para ser usado quando necessario
   if(buffer_index[current_coil] >= (BUFFER_SIZE-1)){//buffer cheio
       //mover o buffer para tras
       for(int i=0;i<(BUFFER_SIZE-1);i++){
           coil_buffer[current_coil][i] = coil_buffer[current_coil][i+1];
       }
       coil_buffer[current_coil][buffer_index[current_coil]] = coil_canal1[current_coil];//coloca o novo valor na ultima posicao
   }

   //print de parte do buffer - para debug
   //fprintf(stderr,"\nParte do Buffer da coil(0) = %d %d %d %d %d %d %d %d %d %d",coil_buffer[0][0],coil_buffer[0][1],coil_buffer[0][2],coil_buffer[0][3],coil_buffer[0][4],coil_buffer[0][5],coil_buffer[0][6],coil_buffer[0][7],coil_buffer[0][8],coil_buffer[0][9]);


}

void calcularModa(){

    if(buffer_index[current_coil] == (BUFFER_SIZE-1) && calculou_moda[current_coil] == false)//so analisa depois do buffer cheio
    {
        //calcula a moda
        calculou_moda[current_coil] = true; //para fazer isto só uma vez (para cada coil)
        
        //forcar as inexistentes
		//calculou_moda[1] = true; 
		//calculou_moda[2] = true; 

        //Analisar desvio padrao do buffer
        int  media, desviopadrao;
        int j, tam;
        tam=BUFFER_SIZE;
        /* Calcula a media do vetor */
        for(media=0.0, j=0; j<=(tam-1); j++)
        media += coil_buffer[current_coil][j];
        media /= (float) tam;

        fprintf(stderr,"\nMedia=%d" , media);

        //convert the buffer to float
        float float_buffer[BUFFER_SIZE];
        for(int i=0;i<BUFFER_SIZE;i++){
          float_buffer[i] = (float)coil_buffer[current_coil][i];
        }
        //fprintf(stderr,"\nMean_function=%f" , mean_function(float_buffer,BUFFER_SIZE));
        //fprintf(stderr,"\nMedian_function=%f" , median_function(float_buffer,BUFFER_SIZE));
        moda_ou_zero[current_coil] = mode_function(float_buffer,BUFFER_SIZE);
        fprintf(stderr,"\n --- Mode_function para coil %d = %d" , current_coil, moda_ou_zero[current_coil]);


        /* Calcula o desviopadrao do vetor */
        for(desviopadrao=0.0, j=0; j<=(tam-1); j++)
        //variancia += (coil_buffer[j]-media)*(coil_buffer[j]-media);
        desviopadrao += abs(coil_buffer[current_coil][j]-media);
        desviopadrao /= (float) tam;
        /* Mostra os resultados na tela */
        // printf(" Media = %d \n", media);
        // printf(" DP= %d", desviopadrao);
    }
}

void normalizarCoils(){
   
   //para 3 coils
   // if(moda_ou_zero[0] != 0 && moda_ou_zero[1] != 0 && moda_ou_zero[2] != 0 && flag_calculou_as_modas_uma_vez == false){
      
   //para 1 coil
      if(moda_ou_zero[0] != 0 && flag_calculou_as_modas_uma_vez == false){
         //normalizar as coils
        //encontrar a menor moda --------------------------------------------------
        int menor_moda = 9999999;
        //moda_ou_zero[0];
        
        int menor_moda_index = 0;
        for(int i=0;i<3;i++){
            if(moda_ou_zero[i]<menor_moda){
                menor_moda = moda_ou_zero[i];
                menor_moda_index = i;
            }
        }
        
        fprintf(stderr,"\nMenor moda = %d",menor_moda);
        //diferencas

        ajuste[0] = moda_ou_zero[0] - menor_moda;
        ajuste[1] = moda_ou_zero[1] - menor_moda;
        ajuste[2] = moda_ou_zero[2] - menor_moda;
        //um deles vai ser zero, os outros ajustes vao conter a difereca entre as coils
        fprintf(stderr,"\nAjustes 0=%d   1=%d   2=%d",ajuste[0],ajuste[1],ajuste[2]);
        flag_calculou_as_modas_uma_vez = true;

     }else{
		//fprintf(stderr,"\nNao calculou a moda ainda...");
	 }

}

void msgCallback(const boost::shared_ptr<const metal_detector_msgs::Coil>& coil_ptr)
{
    if(flag_sweep){
        geometry_msgs::PointStamped point_in;
        point_in.header.frame_id = coil_ptr->header.frame_id;
        point_in.header.stamp = coil_ptr->header.stamp;
        point_in.point.x = 0.0;
        point_in.point.y = 0.0;
        point_in.point.z = 0.0;

        geometry_msgs::PointStamped point_out;

		c2.r = 255;
		c2.g = 0;
		c2.b = 0;
		c2.a = 1.0;

        try
        {
            tf_.transformPoint(target_frame_, point_in, point_out);

            // Note that z is the position of the coil, not the position of the possible metal sample!
            /* ROS_INFO("Coil %s with data ch0 %d ch1 %d ch2 %d at x %f y %f z %f",
                coil_ptr->header.frame_id.c_str(),
                coil_ptr->channel[0],
                coil_ptr->channel[1],
                coil_ptr->channel[2],
                point_out.point.x,
                point_out.point.y,
                point_out.point.z);
*/
			 point_out.header.frame_id = coil_ptr->header.frame_id;
             point_out.header.stamp = coil_ptr->header.stamp;


//***DETECCAO DE MINAS ******************
				  
    encherBuffer(coil_ptr);
    calcularModa();

    normalizarCoils();



if(flag_calculou_as_modas_uma_vez){//so analisa depois de calcular as modas
    
   
	
	//esta linha normaliza as coils (aproximadamente)
	coil_canal1[current_coil] = coil_canal1[current_coil] - ajuste[current_coil];
	
    //----------------------------------------------
    //separar as coils aqui tambem, e é a diferença em relacao a moda que interessa, e nao o valor absoluto
    int diff = coil_canal1[current_coil] - (moda_ou_zero[current_coil]-ajuste[current_coil]);
    
    //Debug
    //fprintf(stderr," %d (%d)",diff,personal_mine_limiar);

    //LOW METAL - com buffer_size=18
    //diff > -11000 ---> apanhou a APNM
    //diff > -13300 ---> apanhou a PRBm409 com zero FP --- 13700 apanha 1 fp
    //*********
    //diminuir este valor de 30000 para aumentar a sensibilidade (tambem aumenta os falsos positivos)
	if(diff > personal_mine_limiar && diff < tank_mine_limiar){
		if(!flag_detected){
			fprintf(stderr,"\nANTI-PERSONAL MINE DETECTED ! n_het=%d diff=%d \n",++number_of_heter,diff);
			flag_detected = true;
			nao_detections = 0;
			point_out.point.z = 1;//anti-personal
			//todo guardar posicoes das heterogeneidades em um buffer de pontos
			publishRvizMarker(point_out.point.x,   point_out.point.y,   100,   c2);
			pontos_prado_pub.publish(point_out.point);
		
		}
	}else
	if(diff >= tank_mine_limiar){
		if(!flag2_detected){
			fprintf(stderr,"\nANTI-TANK MINE DETECTED ! n_het=%d diff=%d \n",++number_of_heter,diff);
			flag2_detected = true;
			nao_detections = 0;
			point_out.point.z = 2;//anti-tank
			publishRvizMarker(point_out.point.x,   point_out.point.y,   100,   c2);
			pontos_prado_pub.publish(point_out.point);
		
		}
	}else{
		flag_detected = false;
		flag2_detected = false;
		nao_detections++;
	}
	
// *** juntar as heterogeneidades em possiveis minas -- implementado no outro modulo --- mine_maps
//nao_detections aqui vai ter a ver com o espaço esperado entre cada mina
    if(nao_detections > 1250){ //um sweep sem minas, hora de recalcular as modas

        fprintf(stderr,"\n ------------ Recalibrando... --------------- ");
		
        //se eu quiser calcular a moda novamente
        //buffer_index[0] = buffer_index[1]= buffer_index[2]=0;
        //encherBuffer(coil_ptr);
        calculou_moda[0] = calculou_moda[1] = calculou_moda[2] = false;

        calcularModa();
     
		nao_detections = 0;
        //number_of_heter=0;
		
    }else{

	  //fprintf(stderr," %f ",(coil_canal1[0]/MY_RAW_ESCALA));
		
      //fim da analise
      //so publica se tiver analisado --- isto faz com que não publique o buffer inicial - pois o buffer nao estara normalizado
      //canal 1
      publish_pcl_raw(point_out.point.x,   point_out.point.y,   (coil_canal1[0]/MY_RAW_ESCALA), &pcl_raw_pub);
			
      //canal 0
      //publish_pcl_raw(point_out.point.x+20.0,   point_out.point.y,   (coil_canal0[current_coil]/MY_RAW_ESCALA), &pcl_raw_pub);
      ////publishRvizMarker(point_out.point.x,   point_out.point.y,   (coil_canal0[current_coil]/MY_RAW_ESCALA),   corPonto);
      // pub_.publish( point_out );
    }

}//fim da analise

//*********************************************************************

			//publicar aqui se quiser o buffer inicial
	


        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("Failure %s\n", ex.what());
        }
    }
}//fim do if flag_sweep

}; //fim da classe metal detector


void flag_msgCallback(const std_msgs::String& msg)
{
  if(msg.data == "start")
  {
      ROS_INFO("Sweep initiated: acquiring data from the metal detector");
      flag_sweep=true;
  }
  else if(msg.data == "stop")
  {
       ROS_INFO("Sweep paused: ignoring data from the metal detector");
       flag_sweep=false;
  }

}

 int main(int argc, char **argv)
 {
   fprintf(stderr,"\nStarted...\n");

   ros::init(argc, argv, "metal_detector_viewer_node");
   ros::NodeHandle nh;
   ros::Rate r(10);
    
   // Get parameters
   if(nh.getParam("/metal_detector_viewer_node/personal_mine_limiar", personal_mine_limiar)){
		fprintf(stderr,"\nParameter personal_mine_limiar was get as %d\n\n",personal_mine_limiar);
   }else{
	    // Default value version
	    nh.param("personal_mine_limiar", personal_mine_limiar, -4150); 
		fprintf(stderr,"\nParameter personal_mine_limiar was set for default as %d\n\n",personal_mine_limiar);
   }
   
   if(nh.getParam("/metal_detector_viewer_node/tank_mine_limiar", tank_mine_limiar)){
		fprintf(stderr,"\nParameter tank_mine_limiar was get as %d\n\n",tank_mine_limiar);
   }else{
	    // Default value version
	    nh.param("tank_mine_limiar", tank_mine_limiar, 300000); 
		fprintf(stderr,"\nParameter tank_mine_limiar was set for default as %d\n\n",tank_mine_limiar);
   }
  
  //RAW point cloud 
   pcl_raw_pub = nh.advertise<sensor_msgs::PointCloud> ("raw_points", 100);
  
   pontos_prado_pub = nh.advertise<geometry_msgs::Point> ("geo_point_minas", 1000);

   boost::thread_group threads;


   //Publisher of marker for rviz
   marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
   //PointCloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> ("points2", 1);

    
	 MetalDetector md;
        
        
   ros::spin();
   
   threads.join_all();

  
 }
