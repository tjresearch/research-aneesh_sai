using System.IO;
using UnityEngine;
using System;
using System.Collections;

public class TakePicture : MonoBehaviour {
    

    public string side;
    public Camera camera;
    public int resWidth=960;
    public int resHeight=540;
    private void LateUpdate()
    {
        if (Input.GetKeyDown(KeyCode.Alpha9))
        {
            CamCapture();  
        }
        
    }
    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Alpha0))
        {
            string countPath=Application.dataPath+"/StereoImages/"+side+"count.txt";
            System.IO.File.WriteAllText(countPath, "1");
            Debug.Log("Reset "+countPath);
        }
    }
    
    void CamCapture()
    {
        string countPath=Application.dataPath+"/StereoImages/"+side+"count.txt";
        int number = Int32.Parse(System.IO.File.ReadAllText(countPath));
        RenderTexture rt = new RenderTexture(resWidth, resHeight, 24);
        camera.targetTexture = rt;
        Texture2D screenShot = new Texture2D(resWidth, resHeight, TextureFormat.RGB24, false);
        camera.Render();
        RenderTexture.active = rt;
        screenShot.ReadPixels(new Rect(0, 0, resWidth, resHeight), 0, 0);
        camera.targetTexture = null;
        RenderTexture.active = null; // JC: added to avoid errors
        Destroy(rt);
        byte[] bytes = screenShot.EncodeToPNG();
        string filename = Application.dataPath + "/StereoImages/" + side + "_" +  number.ToString() + ".png";
        number++;
        System.IO.File.WriteAllText(countPath, number.ToString());
        System.IO.File.WriteAllBytes(filename, bytes);
        Debug.Log(string.Format("Took screenshot to: {0}", filename));

    }
    
}

