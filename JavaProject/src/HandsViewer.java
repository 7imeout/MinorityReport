/*******************************************************************************

 INTEL CORPORATION PROPRIETARY INFORMATION
 This software is supplied under the terms of a license agreement or nondisclosure
 agreement with Intel Corporation and may not be copied or disclosed except in
 accordance with the terms of that agreement
 Copyright(c) 2014 Intel Corporation. All Rights Reserved.

 *******************************************************************************/

import intel.rssdk.*;
import java.lang.System.*;
import java.util.*;
import javax.swing.*;
import java.awt.event.*;
import java.awt.image.*;
import java.awt.*;

public class HandsViewer extends JApplet {
    public static void main(String s[]) {
        // Create session
        PXCMSession session = PXCMSession.CreateInstance();
        if (session == null) {
            System.out.print("Failed to create a session instance\n");
            System.exit(3);
        }

        PXCMSenseManager senseMgr = session.CreateSenseManager();
        if (senseMgr == null) {
            System.out.print("Failed to create a SenseManager instance\n");
            System.exit(3);
        }

        PXCMCaptureManager captureMgr = senseMgr.QueryCaptureManager();
        captureMgr.FilterByDeviceInfo("RealSense", null, 0);

        pxcmStatus sts = senseMgr.EnableHand(null);
        if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0) {
            System.out.print("Failed to enable HandAnalysis\n");
            System.exit(3);
        }

        /*
        PXCMHandModule handModule = senseMgr.QueryHand();
        PXCMHandConfiguration handConfig = handModule.CreateActiveConfiguration();
        handConfig.EnableAllGestures();
        handConfig.EnableAllAlerts();
        handConfig.ApplyChanges();
        handConfig.Update();
        */

        sts = senseMgr.Init();
        //if(sts.isError())
        //System.out.println("Init failed and we get " + sts.toString());
        if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)>=0) {
            PXCMHandModule handModule = senseMgr.QueryHand();
            PXCMHandConfiguration handConfig = handModule.CreateActiveConfiguration();
            handConfig.EnableAllGestures();
            handConfig.EnableAllAlerts();
            //handConfig.EnableTrackedJoints(true);
            //handConfig.EnableNormalizedJoints(true);
            handConfig.ApplyChanges();
            handConfig.Update();


            PXCMHandData handData = handModule.CreateOutput();

            for (int nframes=0; nframes<30000; nframes++) {
                sts = senseMgr.AcquireFrame(true);
                if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR)<0) break;

                PXCMCapture.Sample sample = senseMgr.QueryHandSample();

                // Query and Display Joint of Hand or Palm
                handData.Update();

                System.out.println ("Frame # " + nframes + " Hands: " + handData.QueryNumberOfHands());

                PXCMHandData.IHand hand = new PXCMHandData.IHand();
                sts = handData.QueryHandData(PXCMHandData.AccessOrderType.ACCESS_ORDER_NEAR_TO_FAR, 0, hand);

                /* Mike's code */
                PXCMHandData.JointData jointData = new PXCMHandData.JointData();
                if (hand.HasTrackedJoints()) {
                    for (PXCMHandData.JointType type : PXCMHandData.JointType.values()) {
                        if (hand.QueryTrackedJoint(type, jointData) == pxcmStatus.PXCM_STATUS_NO_ERROR)
                            System.out.println(type.toString()
                                    + "x: " + jointData.positionWorld.x
                                    + "y: " + jointData.positionWorld.y
                                    + "x: " + jointData.positionWorld.z
                            );
                    }
                }


                //if (sts.isError())
                //continue;

                PXCMHandData.JointData data = new PXCMHandData.JointData();


                if (sts.compareTo(pxcmStatus.PXCM_STATUS_NO_ERROR) >= 0) {
                    hand.QueryTrackedJoint(PXCMHandData.JointType.JOINT_CENTER, data);
                    PXCMPointF32 image = hand.QueryMassCenterImage();
                    PXCMPoint3DF32 world = hand.QueryMassCenterWorld();

                    System.out.println("Palm Center at frame " + nframes + ": ");
                    System.out.print("   Image Position: (" + image.x + "," +image.y + ")");
                    System.out.println("   World Position: (" + world.x + "," + world.y + "," + world.z + ")");
                }

                // alerts
                int nalerts = handData.QueryFiredAlertsNumber();
                System.out.println("# of alerts at frame " + nframes + " is " + nalerts);

                // gestures
                int ngestures = handData.QueryFiredGesturesNumber();
                System.out.println("# of gestures at frame " + nframes + " is " + ngestures);

                senseMgr.ReleaseFrame();
            }
            System.exit(0);
        }
    }
}
