/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to read all 64 distance readings at once.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>
#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <lvgl.h>
#include <LilyGo_AMOLED.h>
#include <LV_Helper.h>
#include <micro_ros_platformio.h>
#include <micro_ros_utilities/string_utilities.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/visibility_control.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcl/types.h>
#include <sensor_msgs/msg/image.h>
#include <checks.cpp>

#define SSID "OpenWrt"
#define SSID_PW "dimostratore"

void error_loop(rcl_ret_t rc) {
  Serial.println("Error loop");
  while (1) {
    delay(100);
  }
}

// ROS
rcl_publisher_t publisher;
sensor_msgs__msg__Image msg_img;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_support_t support;

// DEVICES
SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM
LilyGo_Class amoled;

static void btn_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *btn = lv_event_get_target(e);
    if (code == LV_EVENT_CLICKED)
    {
        static uint8_t cnt = 0;
        cnt++;

        /*Get the first child of the button which is the label and change its text*/
        lv_obj_t *label = lv_obj_get_child(btn, 0);
        lv_label_set_text_fmt(label, "asdasdasdasd: %d", cnt);
    }
}

void lv_example_get_started_1(void)
{
    lv_obj_t *btn = lv_btn_create(lv_scr_act());                /*Add a button the current screen*/
    lv_obj_set_pos(btn, 10, 10);                                /*Set its position*/
    lv_obj_set_size(btn, 120, 50);                              /*Set its size*/
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL); /*Assign a callback to the button*/

    lv_obj_t *label = lv_label_create(btn); /*Add a label to the button*/
    lv_label_set_text(label, "Button");     /*Set the labels text*/
    lv_obj_center(label);
}

void setup()
{
  // SERIAL
  Serial.begin(115200);
  delay(1000);
  Serial.println("MicroRos VL53L5CX ");


  // WIFI
  Serial.println("WIFI Setup");
  char ssid[] = SSID;
  char ssid_pw[] = SSID_PW;
  IPAddress agent_ip(192, 168, 1, 66);
  Serial.println("WIFI Setup 1");
  const uint16_t k_agent_port = 8888;
  set_microros_wifi_transports(ssid, ssid_pw, agent_ip, k_agent_port);
  Serial.println("WIFI Setup 2");
  delay(2000);
  Serial.println("WIFI ready");

  // ROS
  Serial.println("ROS setup");
  rcl_allocator_t allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));   // create init_options
  
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support)); // create node

  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
      "micro_lidar"));    // create publisher

  // RCCHECK(rclc_executor_init(&executor, &support.context, 0, &allocator));
  Serial.println("ROS ready");


  // ROS MSG
  // static micro_ros_utilities_memory_conf_t conf = {0};
  // bool success = micro_ros_utilities_create_message_memory(
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
  //   &msg_img,
  //   conf
  // );
  // msg_img.encoding.data = "16UC1";
  // rosidl_runtime_c__String encoding_type;
  msg_img.encoding.data        = (char * ) malloc(5 * sizeof(char));
  msg_img.encoding.data        = (char*)"mono16"; 
	msg_img.encoding.size        = 5;
	msg_img.encoding.capacity    = 6;
  msg_img.height               = 8;
  msg_img.width                = 8;
  msg_img.step                 = 16;
  msg_img.data.data            = (uint8_t*) malloc(2 * 64 * sizeof(char));
  msg_img.data.capacity        = 128;
  msg_img.data.size            = 128;
  msg_img.header.frame_id.data = (char * ) malloc(5 * sizeof(char));
  msg_img.header.frame_id.data = (char*)"ld1"; 



  // I2C lidar
  Wire.begin(43,44);
  Wire.setClock(1000000);
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  if (myImager.begin() == false)
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1) ;
  }
  
  myImager.setResolution(64); 

  bool response = myImager.setRangingFrequency(15);
  if (response == true)
  {
    int frequency = myImager.getRangingFrequency();
    if (frequency > 0)
    {
      Serial.print("Ranging frequency set to ");
      Serial.print(frequency);
      Serial.println(" Hz.");
    }
    else
      Serial.println(F("Error recovering ranging frequency."));
  }
  else
  {
    Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
    while (1) ;
  }

  myImager.startRanging();



  // DISPLAY
  bool rslt = false;
  rslt =  amoled.beginAMOLED_191();

  if (!rslt) {
      while (1) {
          Serial.println("The board model cannot be detected, please raise the Core Debug Level to an error");
          delay(1000);
      }
  }

  beginLvglHelper(amoled);
  lv_example_get_started_1();
}

void loop()
{
  //Poll sensor for new data
  if (myImager.isDataReady() == true)
  {
    if (myImager.getRangingData(&measurementData)) //Read distance data into array
    {
      msg_img.data.data = (uint8_t *)measurementData.distance_mm;
      //The ST library returns the data transposed from zone mapping shown in datasheet
      //Pretty-print data with increasing y, decreasing x to reflect reality
      // for (int y = 0 ; y <= 8 * (8 - 1) ; y += 8)
      // {
        // for (int x = 8 - 1 ; x >= 0 ; x--)
        // {
          // Serial.print("\t");
          // Serial.print(measurementData.distance_mm[x + y]);
          // msg_img.data.data[x+y] = measurementData.distance_mm[x + y];
        // }
        // Serial.println();
      // }
      // Serial.println();
    }
  }

  lv_task_handler();
  RCSOFTCHECK(rcl_publish(&publisher, &msg_img, NULL));
  delay(50); //Small delay between polling
}
