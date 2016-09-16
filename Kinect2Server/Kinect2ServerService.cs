/*******************************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are permitted provided 
that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list of conditions and 
    the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
    and the following disclaimer in the documentation and/or other materials provided with the 
    distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors may be used to 
    endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************************************/
using Microsoft.Kinect;
using System;
using System.Diagnostics;
using System.ServiceProcess;
using System.Windows.Media;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Windows.Media.Imaging;
using System.Windows;
using OpenCvSharp;
using OpenCvSharp.Extensions;
using OpenCvSharp.Blob;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// A service that publishes data from the Kinect2 over TCP sockets.
    /// </summary>
    /// See: http://msdn.microsoft.com/en-us/library/system.serviceprocess.servicebase(v=vs.110).aspx
    /// 
    public class Kinect2ServerService : ServiceBase
    {
        KinectSensor kinect;
        MultiSourceFrameReader reader;
        AudioSource audioSource;
        AudioBeamFrameReader audioReader;

        AsyncNetworkConnector colorConnector;
        AsyncNetworkConnector depthConnector;
        AsyncNetworkConnector irConnector;
        AsyncNetworkConnector bodyConnector;
        AsyncNetworkConnector audioConnector;
        AsyncNetworkConnector scan2DConnector;

        CoordinateMapper mapper;
        //CameraIntrinsics intrinsics = mapper.GetDepthCameraIntrinsics();
        CameraSpacePoint[] cameraSpace = new CameraSpacePoint[512 * 424];
        byte[] colorArray;
        ushort[] depthArray;
        ushort[] irArray;
        float[] scan2DArray;
        byte[] byteScan2DArray;
        byte[] byteColorArray;
        byte[] byteDepthArray;
        byte[] byteIRArray;
        Body[] bodyArray;
        AudioContainer audioContainer;


        static readonly int BYTES_PER_COLOR_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        const int BYTES_PER_DEPTH_PIXEL = 2;
        const int BYTES_PER_IR_PIXEL = 2;

        const float camera_x = 0.0F;
        const float camera_y = 20.0F;
        const float camera_z = 0.0F;
        int target_x = 0;
        int target_y = 0;
        int target_z = 0;

        CvMat lower;
        CvMat upper;
        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;
        private WriteableBitmap colorBitmap = null;

        public Kinect2ServerService()
        {
            this.ServiceName = "Kinect2Server";
            this.CanStop = true;
            this.CanPauseAndContinue = false;
            this.AutoLog = true;


            }

        /// <summary>
        /// Property that indicates whether the Kinect Server is connected to a sensor.
        /// </summary>
        public bool IsConnected { get { return (this.kinect != null) && kinect.IsAvailable; } }

        /// <summary>
        /// Event that triggers when the server detects a Kinect connection or disconnecting.
        /// </summary>
        public event EventHandler<IsConnectedChangedEventArgs> IsConnectedChanged;

        protected override void OnStart(string[] args)
        {
            // Try to open the first available Kinect sensor.
            this.kinect = KinectSensor.GetDefault();
            if (this.kinect == null)
            {
                EventLog.WriteEntry("No Kinect device was detected.");  
                ExitCode = -1;
                throw new KinectException("No kinect device was detected.");
            }
            else
            {
                this.kinect.Open();
                this.kinect.IsAvailableChanged += this.OnAvailableChanged;
            }

            // Register as a handler for the image data being returned by the Kinect.
            this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);
            this.audioSource = this.kinect.AudioSource;
            if (this.reader == null)
            {
                EventLog.WriteEntry("Unable to connect to Kinect data stream.");
                ExitCode = -2;
                throw new KinectException("Unable to connect to Kinect data stream.");
            }
            else
            {
                this.reader.MultiSourceFrameArrived += this.OnFrameArrived;
            }
            if (this.audioSource == null)
            {
                EventLog.WriteEntry("Unable to open audio source on kinect");
                ExitCode = -3;
                throw new KinectException("Unable to connect to kinect audio source");
            }
            else
            {
                this.audioReader = this.audioSource.OpenReader();
                if (this.audioReader == null)
                    Console.WriteLine("Issues with audio reader");
                else
                    this.audioReader.FrameArrived += this.onAudioFrameArrived;
            }


            // Allocate storage for the data from the Kinect.
            this.colorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL)];
            this.depthArray = new ushort[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];
            this.irArray = new ushort[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width];
            this.byteColorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL) + sizeof(double)];
            this.byteDepthArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL + sizeof(double)];
            this.byteIRArray = new byte[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL + sizeof(double)];
            this.bodyArray = new Body[this.kinect.BodyFrameSource.BodyCount];
            this.scan2DArray = new float[this.kinect.DepthFrameSource.FrameDescription.Width * 6 + 3];
            this.byteScan2DArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Width * 6 *4 + 12];
            this.audioContainer = new AudioContainer();
            this.audioContainer.samplingFrequency = 16000;
            this.audioContainer.frameLifeTime = 0.016;
            this.audioContainer.numSamplesPerFrame = (int)(this.audioContainer.samplingFrequency * this.audioContainer.frameLifeTime);
            this.audioContainer.numBytesPerSample = sizeof(float);
            this.audioContainer.audioStream = new float[256];
            
            // Create network connectors that will send out the data when it is received.
            this.colorConnector = new AsyncNetworkConnector(Properties.Settings.Default.RgbImagePort);
            this.depthConnector = new AsyncNetworkConnector(Properties.Settings.Default.DepthImagePort);
            this.irConnector = new AsyncNetworkConnector(Properties.Settings.Default.IrImagePort);
            this.bodyConnector = new AsyncNetworkConnector(Properties.Settings.Default.BodyPort);
            this.audioConnector = new AsyncNetworkConnector(Properties.Settings.Default.AudioPort);
            this.scan2DConnector = new AsyncNetworkConnector(Properties.Settings.Default.Scan2DPort);

            // Open the server connections.
            this.colorConnector.Listen();
            this.depthConnector.Listen();
            this.irConnector.Listen();
            this.bodyConnector.Listen();
            this.audioConnector.Listen();
            this.scan2DConnector.Listen();
            // get the depth (display)extents

            FrameDescription colorFrameDescription = this.kinect.ColorFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = colorFrameDescription.Width;
            this.displayHeight = colorFrameDescription.Height;

            // open the reader for the body frames


            // open the reader for the color frames

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

             mapper = this.kinect.CoordinateMapper;
             lower = new CvMat(3, 1, MatrixType.U8C1);
             upper = new CvMat(3, 1, MatrixType.U8C1);
            // Set specific color threshold for orange.
            lower[0] = 5;
            lower[1] = 170;
            lower[2] = 170;

            upper[0] = 10;
            upper[1] = 255;
            upper[2] = 255;

        }

        protected override void OnStop()
        {
            this.kinect.Close();
            this.colorConnector.Close();
            this.depthConnector.Close();
            this.irConnector.Close();
            this.bodyConnector.Close();
            this.audioConnector.Close();
            this.scan2DConnector.Close();


            this.reader.Dispose(); // TODO: Is this actually necessary?
            this.audioReader.Dispose();
            this.colorConnector.Dispose();
            this.depthConnector.Dispose();
            this.irConnector.Dispose();
            this.bodyConnector.Dispose();
            this.audioConnector.Dispose();
            this.scan2DConnector.Dispose();

        }

        private void calculateScanFromDepth(ushort[] depthArray)
        {

            CoordinateMapper mapper = this.kinect.CoordinateMapper;
            //CameraIntrinsics intrinsics = mapper.GetDepthCameraIntrinsics();
            CameraSpacePoint[] cameraSpace = new CameraSpacePoint[512 * 424];
            mapper.MapDepthFrameToCameraSpace(depthArray, cameraSpace);
           // Console.WriteLine(cameraSpace[512 * 250].X);
            int i = 0, j = 0;
            float min = float.MaxValue, max = float.MinValue;
            int min_index = 0;
            int max_index = 0;
           // float yaw_angle = 0;
            float y = 0;
            float floor_offset = .425f;

            for (i = 0; i < 512; i ++)
            {
                min = float.MaxValue;
                max = float.MinValue;
                min_index = 0;
                max_index = 0;
               // yaw_angle = 125.0f - (70f / 512f) * i; 
                for (j = i; j < 512 *423 + i ; j+= 512)
                {
                    // float dist =(float) Math.Sqrt(Math.Pow(cameraSpace[j].Y, 2) + Math.Pow(cameraSpace[j].X, 2) + Math.Pow(cameraSpace[j].Z, 2));
                    float dist = cameraSpace[j].Z;
                    if (dist < min && depthArray[j] != 0u)
                    {

                        //min = depthArray[j];
                        // min_angle = 30f - j/512 * (60f / 424f);
                        // y = depthArray[j] * (float)Math.Sin(min_angle * Math.PI / 180) + floor_offset;
                        y = cameraSpace[j].Y + floor_offset;
                        if (y > 0.050 && y < 2.400)
                        {
                            min = cameraSpace[j].Z;
                            min_index = j;
                          
                        }
                    }
                    if (dist > max)
                    {
                        // max = depthArray[j];
                        // max_angle = 30f - j/512 * (60f / 424f);
                        y = cameraSpace[j].Y + floor_offset;// depthArray[j] * (float)Math.Sin(max_angle * Math.PI / 180) + floor_offset;
                        if (y > .050 && y < 2.000)
                        {
                            max = cameraSpace[j].Z;
                            max_index = j;
                        }
                    }
                }
                scan2DArray[6*i] =  min;
                scan2DArray[6 * i + 1] = cameraSpace[min_index].X;
                scan2DArray[6 * i + 2] = cameraSpace[min_index].Y;
                scan2DArray[6 * i + 3] = max;
                scan2DArray[6 * i + 4] = cameraSpace[max_index].X;
                scan2DArray[6 * i + 5] = cameraSpace[max_index].Y;
            }
             cameraSpace = new CameraSpacePoint[1920 * 1080];

            mapper.MapColorFrameToCameraSpace(depthArray, cameraSpace);
            CameraSpacePoint p = cameraSpace[target_y * 1920 + target_x];
            Debug.Print("Cone (x,y,z) is (" + p.X.ToString() + ", " + p.Y.ToString() + ", " + p.Z.ToString() + ")\n");
            scan2DArray[6 * 512] = p.X;
            scan2DArray[6 * 512+1] = p.Y;
            scan2DArray[6 * 512+ 2] = p.Z;
            target_y = 0;
            target_z = 0;
            target_x = 0;
        



        }
        private void OnFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();

            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
                {
                    if (colorFrame != null)
                {
                    colorFrame.CopyConvertedFrameDataToArray(this.colorArray, ColorImageFormat.Bgra);
                   // System.Buffer.BlockCopy(this.colorArray, 0, this.byteColorArray,0,(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL));
                   // System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteColorArray, (this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL), sizeof(double));
                   // this.colorConnector.Broadcast(this.byteColorArray);

                        FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                        using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                        {
                            this.colorBitmap.Lock();

                            // verify data and write the new color frame data to the display bitmap
                            if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                            {
                                colorFrame.CopyConvertedFrameDataToIntPtr(
                                    this.colorBitmap.BackBuffer,
                                    (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                    ColorImageFormat.Bgra);

                                this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                            }

                            this.colorBitmap.Unlock();
                        }
                        
                    
                        IplImage imgSrc = this.colorBitmap.ToIplImage(); // Source, BGR image.
                        IplImage imgGray = new IplImage(imgSrc.Size, BitDepth.U8, 1); // Binary image that has the blobs.
                        IplImage imghsv = new IplImage(imgSrc.Size, BitDepth.U8, 3); // HSV image for thresholding.
                    

                        Cv.CvtColor(imgSrc, imghsv, ColorConversion.BgrToHsv);
                        

                        Cv.InRange(imghsv, lower, upper, imgGray);
                        CvBlobs blobs = new CvBlobs();
                        blobs.Label(imgGray);
                        int min_area = 1500;
                        foreach (KeyValuePair<int, CvBlob> item in blobs)
                        {
                            int label = item.Key;
                            CvBlob blob = item.Value;
                            blob.CalcCentroid();
                            int val = blob.Area;
                            if (val > min_area)
                            {
                                min_area = val;
                                float x = (float)blob.Centroid.X;
                                float y = (float)blob.Centroid.Y;
                                target_x = (int)x;
                                target_y = (int)y;
                            }


                             //   Debug.Print( "Coordinates" + (blob.Centroid.ToString()) +"Area" + val.ToString());


                        }
                        /*
                        IplImage render = new IplImage(imgSrc.Size, BitDepth.U8, 3);
                        blobs.RenderBlobs(imgSrc, render);

                         using (new CvWindow("Orange Blob Detection", WindowMode.AutoSize, render))
                         {
                             CvWindow.WaitKey(0);
                         }
                    */
                    
                    }
                
                if (depthFrame != null)
                {
                   // Debug.Print("HELLO");

                    depthFrame.CopyFrameDataToArray(this.depthArray);
                    System.Buffer.BlockCopy(this.depthArray, 0, this.byteDepthArray, 0, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteDepthArray, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL,sizeof(double));
                   // Console.WriteLine("depth "+ this.byteDepthArray.Length);
                    calculateScanFromDepth(this.depthArray);
                    System.Buffer.BlockCopy(this.scan2DArray, 0, this.byteScan2DArray, 0, 6 * this.kinect.DepthFrameSource.FrameDescription.Width * 4 + 12);
                    // System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteScan2DArray, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * 4, sizeof(double));

                    this.scan2DConnector.Broadcast(this.byteScan2DArray);
                    this.depthConnector.Broadcast(this.byteDepthArray);
                }
                
            }
                }

           

            /*using (InfraredFrame irFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
            {
                if (irFrame != null)
                {
                    irFrame.CopyFrameDataToArray(this.irArray);
                    System.Buffer.BlockCopy(this.irArray, 0, this.byteIRArray, 0, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteIRArray, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL, sizeof(double));
                    this.irConnector.Broadcast(this.byteIRArray);
                }
            }

            using (BodyFrame bodyFrame = multiSourceFrame.BodyFrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    bodyFrame.GetAndRefreshBodyData(this.bodyArray);
                    string jsonString = JsonConvert.SerializeObject(this.bodyArray);
                    int diff = 28000 - jsonString.Length;
                    for (int i = 0; i < diff;i++ )
                    {
                        jsonString += " ";
                    }
                    byte[] bodyByteArray = new byte[jsonString.Length*sizeof(char) + sizeof(double)];
                    System.Buffer.BlockCopy(jsonString.ToCharArray(), 0, bodyByteArray, 0, jsonString.Length * sizeof(char));
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, bodyByteArray, jsonString.Length * sizeof(char),sizeof(double));
                    this.bodyConnector.Broadcast(bodyByteArray);
                }
            }*/
        }

        private void onAudioFrameArrived(object sender,AudioBeamFrameArrivedEventArgs e)
        {
            AudioBeamFrameReference audioFrameRefrence = e.FrameReference;
            try
            {
                AudioBeamFrameList frameList = audioFrameRefrence.AcquireBeamFrames();
                if (frameList != null)
                {
                    using (frameList)
                    {
                        IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;

                        foreach (AudioBeamSubFrame subFrame in subFrameList)
                        {
                            this.audioContainer.utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
                            this.audioContainer.beamAngle = subFrame.BeamAngle;
                            this.audioContainer.beamAngleConfidence = subFrame.BeamAngleConfidence;
                            byte[] array = new byte[this.audioSource.SubFrameLengthInBytes];
                            subFrame.CopyFrameDataToArray(array);
                            for (int i = 0; i < array.Length;i+=sizeof(float))
                            {
                                audioContainer.audioStream[(int)(i / sizeof(float))] = BitConverter.ToSingle(array, i);
                            }
                            string jsonString = JsonConvert.SerializeObject(this.audioContainer);
                            int diff = 4100 - jsonString.Length;
                            for (int i = 0; i < diff;i++)
                            {
                                jsonString += " ";
                            }
                            byte[] transmittedData = new byte[jsonString.Length*sizeof(char)];
                            System.Buffer.BlockCopy(jsonString.ToCharArray(), 0, transmittedData, 0, transmittedData.Length);
                            this.audioConnector.Broadcast(transmittedData);
                            subFrame.Dispose();
                        }
                    }
                    frameList.Dispose();
                }
            }
            catch
            {
            }
        }

        protected void OnAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.IsConnectedChanged(this, new IsConnectedChangedEventArgs(e.IsAvailable));
        }
    }

    /// <summary>
    /// An exception indicating that a Kinect was not detected.
    /// </summary>
    public class KinectException : Exception
    {
        public KinectException()
        {
        }

        public KinectException(string message)
            : base(message)
        {
        }

        public KinectException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }

    /// <summary>
    /// Event triggered where the server connects or disconnects from a Kinect.
    /// </summary>
    public class IsConnectedChangedEventArgs : EventArgs
    {
        bool isConnected;
        public IsConnectedChangedEventArgs(bool isConnected)
        {
            this.isConnected = isConnected;
        }

        public bool IsConnected { get { return isConnected; } }
    }
}
