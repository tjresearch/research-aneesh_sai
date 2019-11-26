using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraMovement : MonoBehaviour
{
    public float movementSpeed=2.0f;
    public float rotationSpeed=1.0f;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if(Input.GetKey(KeyCode.UpArrow))
        {
            transform.Translate(Vector3.forward * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.DownArrow))
        {
            transform.Translate(Vector3.back * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.LeftArrow))
        {
            transform.Translate(Vector3.left * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.RightArrow))
        {
            transform.Translate(Vector3.right * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.Z))
        {
            transform.Translate(Vector3.up * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.X))
        {
            transform.Translate(Vector3.down * (Time.deltaTime * movementSpeed), Space.World);
        }
        if(Input.GetKey(KeyCode.C))
        {
            transform.Rotate(-rotationSpeed,0,0);
        }
        if(Input.GetKey(KeyCode.V))
        {
            transform.Rotate(rotationSpeed,0,0);
        }
        
        
    }
}
