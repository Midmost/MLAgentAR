using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;


/// <summary>
/// A hummingbird machin learning agent
/// </summary>
public class HummingbirdAgent : Agent
{
    [Tooltip("Force to apply when moving")]
    public float moveForce = 2f;

    [Tooltip("SPEED TO PITCH UP OR DOWN")]
    public float pitchSpeed = 100f;

    [Tooltip("Speed to rotate around the up axis")]
    public float yawSpeed = 100f;

    [Tooltip("Transform at the tip of the beak")]
    public Transform beaktip;


    [Tooltip("The agent's camera")]
    public Camera agentCamera;

    [Tooltip("Whether this is training mode or gameplay mode")]
    public bool trainingMode;


    //This rigidbody of the agent
    private Rigidbody rigidbody; //unity used to have a way to quickly look up the rigidbodyon an object, you can't use keyword

    //The flower area that the agent is in
    private FlowerArea flowerArea;

    //The nearest flower to the agent
    private Flower nearestFlower;

    //Allower for smoother pitch changes
    private float smoothPitchChange = 0f;

    //Allows for smoother yar changes
    private float smoothYawChange = 0f; //thie really help to makethe movements more natural

    //Maximum angle that the bird can pitch up or down
    private const float MaxPitchAngle = 80f;  //all the way will be not up 90 degree

    //Maximum distance from the beak tip to accept nectar collision
    //just want to use the last part of the tip when it is inside of nectar because the bird have three part of collider
    //so make sure that if this bird colides with the nectar at all, we only accept that the bird is driinking
    //if the collision happens and the tip of the beak is within this small radius of the vector.
    private const float BeakTipRadius = 0.008f;

    //Whether the agent is frozen (intentionally not flying)
    private bool frozen = false;


    /// <summary>
    /// The amount of nectar the agent has obtained this episode
    /// </summary>
    public float NectarObtained { get; private set; }

    /// <summary>
    /// Initialize the agent
    /// </summary>
    public override void Initialize()
    {
        rigidbody = GetComponent<Rigidbody>();
        flowerArea = GetComponent<FlowerArea>();

        //If not trainig mode, no max step, play forever
        if (!trainingMode) MaxStep = 0;
    }
    /// <summary>
    /// Reset the agent when an episode begins
    /// </summary>
    public override void OnEpisodeBegin()
    {
        if (trainingMode)
        {
            //Only reset flowers in training when there is one agent per area
            flowerArea.ResetFlowers();
        }

        //Reset nectar obtained
        NectarObtained = 0f;

        //Zero out velocities so that movement stops before a new episode begins
        rigidbody.velocity = Vector3.zero;
        rigidbody.angularVelocity = Vector3.zero; //this super important when you're working with ML Agents
        //because when I training the birdis going to be moving with forve in a direction and possible turning
        //when I reset, the world hasn't actually reset still keep going, 
        //So even if I change the position of the bird then it;s goig to keep moving at the same force that it was

        //Default to spwning in frond of a flower
        //we want to give enough ideal or dis-ideal conditions to the agent.
        bool inFrontOfFlower = true;
        if (trainingMode)
        {
            //Spawn in front of flower 50% of the time during trainig
            inFrontOfFlower = UnityEngine.Random.value > 5f; 
        }

        // Move the agent to a new random position
        MoveToSafeRandomPosition(inFrontOfFlower);

        //Recalculate the neares flower now that the agent has moved
        UpdateNearestFlower();

    }

    /// <summary>
    /// Called when and action is received from either the player input of the neural network
    /// 
    /// vectorAction[i] represents:
    /// Index 0 : moce vector x (+1 = right, -1 = move to the left)
    /// Index 1 : move vector y (+1 = up, -1 = down)
    /// Index 2 : moce vector z (+1 = forward, -1 = backward)
    /// Index 3 : pitch angle(+1 = pitch up, -1 = pitch down)
    /// Index 4 : yaw angle(+1 = turn right, -1 = turn left)
    /// 
    /// </summary>
    /// <param name="vectorAction">The actions to take</param>
    public override void OnActionReceived(float[] vectorAction)
    {

        // What we're goint to do is convert these numbers into actual movement in this functions.
        //neural network will learn on its own what to set these values to get the behavior.


        //Don't take actions if frozen 아무짓도 하지마라
        if (frozen) return;

        //Calculate movememt vector
        Vector3 move = new Vector3(vectorAction[0], vectorAction[1], vectorAction[2]);

        //Add force in thte direction of the move vector
        rigidbody.AddForce(move * moveForce);

        //Get the current rotation
        Vector3 rotationVector = transform.rotation.eulerAngles;

        //Calculate pitch and yaw rotation
        float pitchChange = vectorAction[3];
        float yawChange = vectorAction[4];

        //Calculate smooth rotation changes 여기가 중요해
        smoothPitchChange = Mathf.MoveTowards(smoothPitchChange, pitchChange, 2f * Time.fixedDeltaTime);   
        smoothYawChange = Mathf.MoveTowards(smoothYawChange, yawChange, 2f * Time.fixedDeltaTime);
        // 왜 fixedDeltaTime을 썼냐면 fixed update마다 쓰고 싶어서 framerate is variable It completely depends of your computer or whatever.

        //Calculate new pitch and yaw based on smoothed values
        //Clamp pitch to avoid flipping upside down
        float pitch = rotationVector.x + smoothPitchChange * Time.fixedDeltaTime * pitchSpeed;
        float yaw = rotationVector.y + smoothYawChange * Time.fixedDeltaTime * yawSpeed;    // pithc 방향은 제한이 필요하지만 yaw은 필요 없다
        if (pitch > 180f) pitch -= 360f;
        pitch = Mathf.Clamp(pitch, -MaxPitchAngle, MaxPitchAngle);

        //Apply the new rotation
        //회전 대응 각도
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
    }
    /// <summary>
    /// Collect vector obsercations from the environment
    /// </summary>
    /// <param name="sensor">The vector sensor</param>

    public override void CollectObservations(VectorSensor sensor)
    {

        if (nearestFlower == null)
        {

            sensor.AddObservation(new float[10]);
            return;
        }
        //localRotation과 nearestFlower의 상대 위치 관측
        //Obserce the agent's local rotation (4 observations)
        sensor.AddObservation(transform.localRotation.normalized);  //분석을 간소화하기 위해 normalzied쓰고

        //Get a vector from the beaktip tot the nearest flower
        Vector3 toFlower = nearestFlower.FlowerCenterPosition - beaktip.position;


        //Observe a normalized vector pointing to the neareset flower(3 observations)
        sensor.AddObservation(toFlower.normalized);  

        // 벡터곱(내적)을 통해 새 부리가 꽃술 바로 앞에 있는지 관측 (1개 탐지 목표)
        //observe a dot product that indicates whether the bak tip is in front of the flower
        //(+1은 꽃 바로 앞에, -1은 꽃 바로 뒤)
        //(+1 means that the beak tip is directly in front of the flower, -1 means directly behind)
        sensor.AddObservation(Vector3.Dot(toFlower.normalized, -nearestFlower.FlowerUpVector.normalized));


        //the beak 가 꽃을 pointing하고 있는지 아닌지 가리키는 내적을 관측 (관측할 거는 1개만 있는 거임)
        //(+1 means that the beak is pointing directly at the flower, -1 means directly away )
        sensor.AddObservation(Vector3.Dot(beaktip.forward.normalized, -nearestFlower.FlowerUpVector.normalized));


        //Observe the relative distance from the beak tip to the flower(1observation)
        //부리부터 꽃 까지 상대적인 거리를 측정(관측할 게 1개 밖에 없는거임)
        sensor.AddObservation(toFlower.magnitude / FlowerArea.AreaDiameter);

 
    }

    /// <summary>
    /// when Behavior type is set to “Heuristic Only”,on the agent's Behavior Parameters, 
    /// this function will be called. Its return values will be fed into
    /// </summary>
    /// <param name="actionsOut">instaed of using the neural network</param>
    /// 

    //  여기서 휴리스틱은 뉴럴넷 결정과 별개로 결정내리는 또 다른 방식을 말하는거임 약간 대안이랄까...사람이 직접 움직여주는...노가다..
    public override void Heuristic(float[] actionsOut)
    {
        //휴리스틱으로 하면 좀 새 움직임이 이상할 수는 있음
        //WASD와 화살표키를 동시에 써야 움직임이 자연스럽다고 한다...네....


        // create placeholders for all movement/turning
        Vector3 forward = Vector3.zero;
        Vector3 left = Vector3.zero;
        Vector3 up = Vector3.zero;
        float pitch = 0f;
        float yaw = 0f;

        // Conver keyboard inputs to movement and turning
        // all values shoud be between（ -1 ， +1 ）

        // Forward/backward
        if (Input.GetKey(KeyCode.W)) forward = transform.forward;
        else if (Input.GetKey(KeyCode.S)) forward = -transform.forward;

        //Left/right
        if (Input.GetKey(KeyCode.A)) left = -transform.right;
        else if (Input.GetKey(KeyCode.D)) left = transform.right;

        //Up/down
        if (Input.GetKey(KeyCode.E)) up = transform.up;
        else if (Input.GetKey(KeyCode.Q)) up = -transform.up;

        //Pitch up/down
        if (Input.GetKey(KeyCode.UpArrow)) pitch = 1f;
        else if (Input.GetKey(KeyCode.DownArrow)) pitch = -1f;

        //Turn lef/right
        if (Input.GetKey(KeyCode.LeftArrow)) yaw = -1f;
        else if (Input.GetKey(KeyCode.RightArrow)) yaw = 1f;

        //combine the movement vectors and normalize
        Vector3 combined = (forward + left + up).normalized;

        // Add the 3 movement values, pitch, and yaw to the actionsOUt array
        actionsOut[0] = combined.x;
        actionsOut[1] = combined.y;
        actionsOut[2] = combined.z;
        actionsOut[3] = pitch;
        actionsOut[4] = yaw;
    }

    /// <summary>
    /// Prevent the agent from moving and taking actions
    /// </summary>
    public void FreezeAgent()
    {

        //훈련모드에서 freeze를 지원하지 않음
        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = true;
        rigidbody.Sleep();  //rigidbody 멈춤 SWIFT에서 오늘 썼었는데...똑같은 함수네...
    }
    /// <summary>
    /// reSUME AGENT MOVEMENT AND ACTIONS
    /// </summary>
    public void UnfreezeAgent()
    {

        Debug.Assert(trainingMode == false, "Freeze/Unfreeze not supported in training");
        frozen = false;
        rigidbody.WakeUp();
    }





    /// <summary>
    /// Move the agent to a safe random position (i.e does not collide with anything)
    /// if in fromt of flower, also point the beak at the flower
    /// </summary>
    /// <param name="inFrontOfFlower">whether to choose a spot in frond of a flower</param>
    private void MoveToSafeRandomPosition(bool inFrontOfFlower)
    {
        bool safePositionFound = false;
        int attemptsRemaining = 100; //prevent an infitie loop
        Vector3 potentialPosition = Vector3.zero;
        Quaternion potentialRotation = new Quaternion();

        //Loop until a safe position is found or we run out of attempts
        while(!safePositionFound && attemptsRemaining > 0)
        {
            attemptsRemaining--;
            if (inFrontOfFlower)
            {
                //Pick a random flower
                Flower randomFlower = flowerArea.Flowers[UnityEngine.Random.Range(0, flowerArea.Flowers.Count)];

                //position 10 to 20 cm in front of the flower
                float distanceFromFlower = UnityEngine.Random.Range(.1f, .2f);
                potentialPosition = randomFlower.transform.position + randomFlower.FlowerUpVector * distanceFromFlower;

                //point beak at flower (bird's head is center of transform)
                Vector3 toFlower = randomFlower.FlowerCenterPosition - potentialPosition;
                potentialRotation = Quaternion.LookRotation(toFlower, Vector3.up); 
            }
            else
            {
                //pick a random height from the ground
                float height = UnityEngine.Random.Range(1.2f, 2.5f);
                 
                //Pick a random radius from the center of the area
                float radius = UnityEngine.Random.Range(2f, 7f);

                //Pick a random direction rotated around the y axis
                Quaternion direction = Quaternion.Euler(0f, UnityEngine.Random.Range(-180f, 180f), 0f);

                //combine height, radius, and direction to pick a potential position
                potentialPosition = flowerArea.transform.position + Vector3.up * height + direction * Vector3.forward * radius;

                //Choose and ser random starting pitch and yaw
                float pitch = UnityEngine.Random.Range(-60f, 60f);
                float yaw = UnityEngine.Random.Range(-180f, 180f);
                potentialRotation = Quaternion.Euler(pitch, yaw, 0f);
            
            }

            //Check to see if the agent will collide with anything
            Collider[] colliders = Physics.OverlapSphere(potentialPosition, 0.05f);

            //safe position has been found if no colliders are overlapped
            safePositionFound = colliders.Length == 0;
        }

        Debug.Assert(safePositionFound, "Could not find a safe positioiin to spawn");

        //Set the ppositon and rotation
        transform.position = potentialPosition;
        transform.rotation = potentialRotation;
    }

    /// <summary>
    /// Update the nearest flower to the agent
    /// </summary>
    private void UpdateNearestFlower()
    {

        foreach (Flower flower in flowerArea.Flowers)
        {

            //if this not match go to next UPDATE and CHECKING
            if (nearestFlower == null && flower.HasNectar)
            {
                //No current nearest flower and this flower has nectar, so set to this flower
                nearestFlower = flower;
            }
            else if (flower.HasNectar)
            {

                //calculate distance to this flwoer and distance to the current nearest flower 
                float distanceToFlower = Vector3.Distance(flower.transform.position, beaktip.position);
                float distanceToCurrentNearestFlower = Vector3.Distance(nearestFlower.transform.position, beaktip.position); //for more persise
                //now, we need to figure out which one is closer

                //If furrent nearest flower is empty OR this flwoer is closer, update the nearest flower
                if (!nearestFlower.HasNectar || distanceToCurrentNearestFlower > distanceToFlower)
                {

                    nearestFlower = flower;
                }

            }
        }
    }



    /// <summary>
    /// Called when the agent's collider engters a trigger collider
    /// </summary>
    /// <param name="other">The tirgger collider</param>
    private void OnTriggerEnter(Collider other)
    {

        TriggerEnterOrStay(other);
    }

    /// <summary>
    /// Called when the agent's collider stays in a trigger collider
    /// </summary>
    /// <param name="other"></param>
    private void OnTriggerStay(Collider other)
    {

        TriggerEnterOrStay(other);
    }

    /// <summary>
    /// Hadbles when the agent's colldier enters of stays in a trigger collider
    /// </summary>
    /// <param name="collider"></param>
    private void TriggerEnterOrStay(Collider collider)
    {
        //check if agent is colliding with nectar
        if (collider.CompareTag("nectar"))
        {

            Vector3 closestPointToBeakTip = collider.ClosestPoint(beaktip.position);// 꽃술 COLLIDER에서 tip에 가장 가까운 점으로 되돌아가기

            // check if the closest collistion point is closde to the beak tip
            // Note: a colllisiton with anything but th ebeka tip should not count
            if (Vector3.Distance(beaktip.position, closestPointToBeakTip) < BeakTipRadius)
            {

                // Look up the flower for this nectar collider
                Flower flower = flowerArea.GetFlowerFromNectar(collider); //계속 인터랙션을 주면서 찾아야 하는거임

                // Attempt to take .01 nectar
                // Note: this is per fixed timestep, meaning it happens every .02 seconds, or 50x per second
                float nectarReceived = flower.Feed(.01f);

                //  keep track of nectar obtained(계속 추적하는 이유는 트레이닝 안 된 경우 때문에 UI를 먼저 보여줘야 해서)
                NectarObtained += nectarReceived;

                if (trainingMode)
                {

                    // Calculate rewuad for getting nectar
                    float bonus = .02f * Mathf.Clamp01(Vector3.Dot(transform.forward.normalized, -nearestFlower.FlowerUpVector.normalized));
                    AddReward(.01f + bonus);        //bonus 밖에서는 새 부리가 꽃술에 닿으면 0.01의 extra 인센티브를 보장한다
                }

                // if flower is empty, update theh neares flower
                if (!flower.HasNectar)
                {

                    UpdateNearestFlower();
                }
            }
        }
    }
    /// <summary>
    /// Called when the agent collides with something solid
    /// </summary>
    /// <param name="collision">THe collisiton info</param>
    private void OnCollisionEnter(Collision collision)
    {

        if (trainingMode && collision.collider.CompareTag("boundary"))
        {

            //collied wiht the area boundaray, give a negative reward
            AddReward(-.5f);
            //그저 터치될 떄 마다 -5가 되게 하고 싶을 뿐!
            //여기는 좀 더 디벨롭 하고 싶으면 커스터마이징 하는 게 좋다고 한다
        }
    }

    /// <summary>
    /// Called every frame
    /// </summary>
    private void Update()
    {

        // Draw a line from the veak tip to the neares flower
        if (nearestFlower != null)
            Debug.DrawLine(beaktip.position, nearestFlower.FlowerCenterPosition, Color.green); //game mode에서는 안 보임
    }

    //ㅋㅋㅋㅋㅋㅋㅋㅋㅋ update에서 버그 있는 거 발견하고 fixed update 추가하심
    //만약 새가 꽃에 집중하면 모든 주변에 있는 꽃들의 꽃술을 얻기 전까진 절대 nearestflower()를 업데이트 안 할거임
    //it cant' feed from it cuz there's no nectar left

    //그래서 fixed update 추가

    /// <summary>
    /// Called every .02 seconds
    /// </summary>
    private void FixedUpdate()
    {
        //Avoid scenario where neares flower nectoar is stolen by opponent and not uptated
        if (nearestFlower != null && !nearestFlower.HasNectar)
            UpdateNearestFlower();
    }

}
