using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace xarm_csharp_demo
{
    class Program
    {
        static void Main(string[] args)
        {
            int ret;
            int state = -1;

            XArmAPIWrapper wrap1 = new XArmAPIWrapper("192.168.1.101");
            XArmAPIWrapper wrap2 = new XArmAPIWrapper("192.168.1.81");
            
            ret = wrap1.get_state(ref state);
            Console.WriteLine("get_state: ret={0}, state={1}", ret, state);
            ret = wrap2.get_state(ref state);
            Console.WriteLine("get_state: ret={0}, state={1}", ret, state);

            // int arm = XArmAPI.create_instance("192.168.1.101", false);
            // Console.WriteLine("create_instance: {0}", arm);
            // ret = XArmAPI.get_state(ref state);
            // Console.WriteLine("get_state: ret={0}, state={1}", ret, state);
            // XArmAPIWrapper wrap = new XArmAPIWrapper(arm);
            // ret = wrap.get_state(ref state);
            // Console.WriteLine("get_state: ret={0}, state={1}", ret, state);

            // float[] pose1 = { 300, 0, 200, 180, 0, 0 };

            // ret = XArmAPI.switch_xarm(arm1);
            // Console.WriteLine("switch_xarm: {0}", ret);
            // ret = XArmAPI.clean_warn();
            // Console.WriteLine("clean_warn: {0}", ret);
            // ret = XArmAPI.clean_error();
            // Console.WriteLine("clean_error: {0}", ret);
            // ret = XArmAPI.motion_enable(true);
            // Console.WriteLine("motion_enable: {0}", ret);
            // ret = XArmAPI.set_mode(0);
            // Console.WriteLine("set_mode: {0}", ret);
            // ret = XArmAPI.set_state(0);
            // Console.WriteLine("set_state: {0}", ret);

            // XArmAPI.reset(true);
            // // ret = XArmAPI.set_position(pose1, true);
            // // Console.WriteLine("set_position: {0}", ret);
            // ret = XArmAPI.move_gohome(0, 0, 0, true);
            // Console.WriteLine("move_gohome: {0}", ret);

            // continuous linear motion
            /*
            float[][] poses = new float[4][];
            poses[0] = new float[] { 300, -100, 100, 180, 0, 0 };
            poses[1] = new float[] { 300, -100, 300, 180, 0, 0 };
            poses[2] = new float[] { 300, 100, 300, 180, 0, 0 };
            poses[3] = new float[] { 300, 100, 100, 180, 0, 0 };
            float radius = 20;
            for (int i = 0; i < 7; i++)
            {
                for (int j = 0; j < poses.Length; j++)
                {
                    float[] pose = poses[j];
                    ret = XArmAPI.set_position(pose, radius, false, 0);
                    Console.WriteLine("set_position: {0}", ret);
                }
            }
            */

            // // continuous joint motion
            // float[][] angles = new float[3][];
            // angles[0] = new float[] { 0, 14, -25, 0, (float)12.9, 0, 0 };
            // angles[1] = new float[] { -14, 40, -75, 0, (float)33.4, (float)-13.8, 0 };
            // angles[2] = new float[] { (float)21.9, 50, -80, 50, 37, 29, 0 };
            // float angle_radius = 60;
            // for (int i = 0; i < 100; i++)
            // {
            //     for (int j = 0; j < angles.Length; j++)
            //     {
            //         float[] angle = angles[j];
            //         ret = XArmAPI.set_servo_angle(angle, 40, 800, 0, false, -1, angle_radius);
            //         Console.WriteLine("set_servo_angle: {0}", ret);
            //     }
            // }


            /*
            ret = XArmAPI.switch_xarm(arm2);
            Console.WriteLine("switch_xarm: {0}", ret);
            ret = XArmAPI.clean_warn();
            Console.WriteLine("clean_warn: {0}", ret);
            ret = XArmAPI.clean_error();
            Console.WriteLine("clean_error: {0}", ret);
            ret = XArmAPI.motion_enable(true);
            Console.WriteLine("motion_enable: {0}", ret);
            ret = XArmAPI.set_mode(0);
            Console.WriteLine("set_mode: {0}", ret);
            ret = XArmAPI.set_state(0);
            Console.WriteLine("set_state: {0}", ret);
            XArmAPI.reset(true);
            ret = XArmAPI.set_position(pose1, true);
            Console.WriteLine("set_position: {0}", ret);
            ret = XArmAPI.move_gohome(0, 0, 0, true);
            Console.WriteLine("move_gohome: {0}", ret);
            */
            /*
            ret = XArmAPI.create_instance(args[0], false);
            Console.WriteLine("create_instance: {0}", ret);
            ret = XArmAPI.clean_warn();
            Console.WriteLine("clean_warn: {0}", ret);
            ret = XArmAPI.clean_error();
            Console.WriteLine("clean_error: {0}", ret);
            ret = XArmAPI.motion_enable(true);
            Console.WriteLine("motion_enable: {0}", ret);
            ret = XArmAPI.set_mode(0);
            Console.WriteLine("set_mode: {0}", ret);
            ret = XArmAPI.set_state(0);
            Console.WriteLine("set_state: {0}", ret);

            byte[] version = new byte[40];
            ret = XArmAPI.get_version(version);
            Console.WriteLine("get_version, ret={0}, version={1}", ret, System.Text.Encoding.ASCII.GetString(version));

            byte[] sn = new byte[40];
            ret = XArmAPI.get_robot_sn(sn);
            Console.WriteLine("get_robot_sn, ret={0}, sn={1}", ret, System.Text.Encoding.ASCII.GetString(sn));

            int state = -1;
            ret = XArmAPI.get_state(ref state);
            Console.WriteLine("get_state, ret={0}, state={1}", ret, state);

            float[] pose = { 0, 0, 0, 0, 0, 0 };
            ret = XArmAPI.get_position(pose);
            Console.WriteLine("get_position, ret={0}, pos=[{1}]", ret, string.Join(", ", pose));
            */

            /*
            int count = 100;
            ret = XArmAPI.set_gripper_enable(true);
            Console.WriteLine("set_gripper_enable, ret={0}", ret);
            ret = XArmAPI.set_gripper_speed(5000);
            Console.WriteLine("set_gripper_speed, ret={0}", ret);
            while (count > 0)
            {
                count -= 1;
                ret = XArmAPI.set_gripper_position(100, true);
                Console.WriteLine("set_gripper_position, ret={0}", ret);
                ret = XArmAPI.set_gripper_position(600, true);
                Console.WriteLine("set_gripper_position, ret={0}", ret);
            }
            */

            /*
            XArmAPI.reset(true);
            float[] pose1 = { 500, 0, 200, 180, 0, 0 };
            ret = XArmAPI.set_position(pose1, true);
            Console.WriteLine("set_position: {0}", ret);

            float[] pose2 = { 500, 200, 200, 180, 0, 0 };
            ret = XArmAPI.set_position(pose2, true);
            Console.WriteLine("set_position: {0}", ret);

            float[] pose3 = { 500, 200, 400, 180, 0, 0 };
            ret = XArmAPI.set_position(pose3, true);
            Console.WriteLine("set_position: {0}", ret);


            float[] pose4 = { 300, 0, 200, 180, 0, 0 };
            ret = XArmAPI.set_position(pose4, true);
            Console.WriteLine("set_position: {0}", ret);

            ret = XArmAPI.move_gohome(0, 0, 0, true);
            Console.WriteLine("move_gohome: {0}", ret);
            */
        }
    }
}
