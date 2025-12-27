using UnityEngine;

/// <summary>
/// Basic robot controller for demonstration in Unity
/// This script demonstrates fundamental concepts for humanoid robot control
/// </summary>
public class RobotController : MonoBehaviour
{
    [Header("Movement Settings")]
    public float moveSpeed = 5.0f;
    public float turnSpeed = 100.0f;

    [Header("Body Parts")]
    public Transform head;
    public Transform leftArm;
    public Transform rightArm;
    public Transform leftLeg;
    public Transform rightLeg;

    [Header("Sensors")]
    public Transform cameraMount; // Where the robot's camera is mounted
    public Transform lidarMount;   // Where the robot's LiDAR is mounted

    private Rigidbody rb;
    private Animator animator;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        animator = GetComponent<Animator>();
    }

    void Update()
    {
        HandleMovement();
        HandleHeadMovement();
        HandleArmMovement();
    }

    /// <summary>
    /// Handle basic movement input (WASD or arrow keys)
    /// </summary>
    private void HandleMovement()
    {
        float horizontalInput = Input.GetAxis("Horizontal");
        float verticalInput = Input.GetAxis("Vertical");

        // Move forward/backward
        Vector3 forwardMovement = transform.forward * verticalInput * moveSpeed * Time.deltaTime;

        // Strafe left/right
        Vector3 rightMovement = transform.right * horizontalInput * moveSpeed * Time.deltaTime;

        // Apply movement
        if (rb != null)
        {
            rb.MovePosition(rb.position + forwardMovement + rightMovement);
        }
        else
        {
            transform.position += forwardMovement + rightMovement;
        }

        // Turning
        float turnInput = Input.GetAxis("Mouse X");
        transform.Rotate(Vector3.up, turnInput * turnSpeed * Time.deltaTime);
    }

    /// <summary>
    /// Handle head movement for looking around
    /// </summary>
    private void HandleHeadMovement()
    {
        if (head != null)
        {
            float verticalLook = Input.GetAxis("Mouse Y");
            // Limit vertical rotation to prevent over-rotation
            verticalLook = Mathf.Clamp(verticalLook, -30f, 30f);

            head.Rotate(-verticalLook, 0, 0);
        }
    }

    /// <summary>
    /// Handle basic arm movements for demonstration
    /// </summary>
    private void HandleArmMovement()
    {
        if (leftArm != null && rightArm != null)
        {
            // Simple arm waving animation with keyboard input
            if (Input.GetKey(KeyCode.Q))
            {
                leftArm.Rotate(Vector3.forward, 2f);
            }
            if (Input.GetKey(KeyCode.E))
            {
                rightArm.Rotate(Vector3.forward, 2f);
            }
        }
    }

    /// <summary>
    /// Get the robot's camera position
    /// </summary>
    public Vector3 GetCameraPosition()
    {
        return cameraMount != null ? cameraMount.position : transform.position + Vector3.up * 1.5f;
    }

    /// <summary>
    /// Get the robot's LiDAR position
    /// </summary>
    public Vector3 GetLiDARPosition()
    {
        return lidarMount != null ? lidarMount.position : transform.position + Vector3.up * 1.2f;
    }

    /// <summary>
    /// Simple animation controller interface
    /// </summary>
    public void SetWalking(bool isWalking)
    {
        if (animator != null)
        {
            animator.SetBool("IsWalking", isWalking);
        }
    }

    /// <summary>
    /// Simple animation controller interface for idle state
    /// </summary>
    public void SetIdle()
    {
        if (animator != null)
        {
            animator.SetBool("IsWalking", false);
            animator.SetBool("IsIdle", true);
        }
    }
}