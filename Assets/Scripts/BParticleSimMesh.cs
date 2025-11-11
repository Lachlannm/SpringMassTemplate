using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;
using UnityEngine.UIElements;

// Check this out we can require components be on a game object!
[RequireComponent(typeof(MeshFilter))]

public class BParticleSimMesh : MonoBehaviour
{
    public struct BSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring
        public int attachedParticle;            // index of the attached other particle (use me wisely to avoid doubling springs and sprign calculations)
    }

    public struct BContactSpring
    {
        public float kd;                        // damping coefficient
        public float ks;                        // spring coefficient
        public float restLength;                // rest length of this spring (think about this ... may not even be needed o_0
        public Vector3 attachPoint;             // the attached point on the contact surface
    }

    public struct BParticle
    {
        public Vector3 position;                // position information
        public Vector3 velocity;                // velocity information
        public float mass;                      // mass information
        public BContactSpring contactSpring;    // Special spring for contact forces
        public bool attachedToContact;          // is thi sparticle currently attached to a contact (ground plane contact)
        public List<BSpring> attachedSprings;   // all attached springs, as a list in case we want to modify later fast
        public Vector3 currentForces;           // accumulate forces here on each step        
    }

    public struct BPlane
    {
        public Vector3 position;                // plane position
        public Vector3 normal;                  // plane normal
    }

    public float contactSpringKS = 1000.0f;     // contact spring coefficient with default 1000
    public float contactSpringKD = 20.0f;       // contact spring daming coefficient with default 20

    public float defaultSpringKS = 100.0f;      // default spring coefficient with default 100
    public float defaultSpringKD = 1.0f;        // default spring daming coefficient with default 1

    public bool debugRender = true;            // To render or not to render


    /*** 
     * I've given you all of the above to get you started
     * Here you need to publicly provide the:
     * - the ground plane transform (Transform)
     * - handlePlaneCollisions flag (bool)
     * - particle mass (float)
     * - useGravity flag (bool)
     * - gravity value (Vector3)
     * Here you need to privately provide the:
     * - Mesh (Mesh)
     * - array of particles (BParticle[])
     * - the plane (BPlane)
     ***/

    public Transform groundPlane;
    public bool handlePlaneCollisions = true;
    public float mass = 1.0f;
    public bool useGravity = true;
    public Vector3 gravity = new Vector3(0f,-9.8f,0f);

    private BParticle[] particles;
    private Mesh mesh;
    private BPlane plane;

    private Vector3[] newVelocities;
    private Vector3[] newPositions;
    

    /// <summary>
    /// Init everything
    /// HINT: in particular you should probbaly handle the mesh, init all the particles, and the ground plane
    /// HINT 2: I'd for organization sake put the init particles and plane stuff in respective functions
    /// HINT 3: Note that mesh vertices when accessed from the mesh filter are in local coordinates.
    ///         This script will be on the object with the mesh filter, so you can use the functions
    ///         transform.TransformPoint and transform.InverseTransformPoint accordingly 
    ///         (you need to operate on world coordinates, and render in local)
    /// HINT 4: the idea here is to make a mathematical particle object for each vertex in the mesh, then connect
    ///         each particle to every other particle. Be careful not to double your springs! There is a simple
    ///         inner loop approach you can do such that you attached exactly one spring to each particle pair
    ///         on initialization. Then when updating you need to remember a particular trick about the spring forces
    ///         generated between particles. 
    /// </summary>
    void Start()
    {
        InitParticles();
        InitPlane();
    }

    void InitParticles()
    {
        // extract vertices from the mesh
        mesh = GetComponent<MeshFilter>().mesh;
        var vertices = mesh.vertices;

        particles = new BParticle[vertices.Length];
        newVelocities = new Vector3[vertices.Length];
        newPositions = new Vector3[vertices.Length];

        // create particles for each vertex
        for (var i = 0;  i < particles.Length; i++)
        {
            var particle = new BParticle();
            particle.position = transform.TransformPoint(vertices[i]);
            particle.velocity = Vector3.zero;
            particle.mass = mass;
            particle.contactSpring = new BContactSpring();
            particle.contactSpring.kd = contactSpringKD;
            particle.contactSpring.ks = contactSpringKS;
            particle.attachedToContact = false;
            particle.attachedSprings = new List<BSpring>();
            particle.currentForces = Vector3.zero;
            particles[i] = particle;
        }

        // set up the springs
        for (var i = 0;  i < particles.Length; i++)
        {
            for (var j = 0; j < particles.Length; j++)
            {
                var p1 = particles[i];
                var p2 = particles[j];

                // don't attach a spring between the same particle
                if (i == j) continue;

                //check if a spring already exists for this connection
                if (p2.attachedSprings.Exists(s => s.attachedParticle == i)) continue;

                //add spring
                var spring = new BSpring();
                spring.attachedParticle = j;
                spring.kd = defaultSpringKD;
                spring.ks = defaultSpringKS;
                spring.restLength = (p1.position - p2.position).magnitude;
                p1.attachedSprings.Add(spring);
            }
        }
    }

    void InitPlane()
    {
        plane = new BPlane();
        plane.normal = groundPlane.transform.rotation * Vector3.up;
        plane.position = groundPlane.position;
    }
    
    
    void UpdateMesh()
    {
        var newVertices = new Vector3[particles.Length];
        for (var i = 0; i < particles.Length; i++)
        {
            newVertices[i] = transform.InverseTransformPoint(particles[i].position);
        }
        mesh.vertices = newVertices;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
    }

    void SimulationStep()
    {
        var delta = Time.fixedDeltaTime;

        // calculate new velocity and position
        for (var i = 0; i < particles.Length; i++)
        {
            CalculateForces(i);

            var p = particles[i];
            var accel = p.currentForces / p.mass;

            newVelocities[i] = p.velocity + accel * delta;
            newPositions[i] = p.position + newVelocities[i] * delta;
        }

        // apply the new values
        for (var i = 0; i < particles.Length; i++)
        {
            var p = particles[i];
            p.velocity = newVelocities[i];
            p.position = newPositions[i];
            particles[i] = p;
        }
    }
    
    void CalculateForces(int index)
    {
        var p = particles[index];
        
        //gravity
        if (useGravity)
        {
            p.currentForces += p.mass * gravity;
        }

        //ground
        if (handlePlaneCollisions)
        {
            if (Vector3.Dot(p.position - plane.position, plane.normal) < 0)
            {
                if (!p.attachedToContact)
                {
                    p.attachedToContact = true;
                    var distance = Vector3.Dot(p.position - plane.position, plane.normal);
                    p.contactSpring.attachPoint = p.position - distance * plane.normal;
                }
            }
            else
            {
                p.attachedToContact = false;
            }

            if (p.attachedToContact)
            {
                //spring force
                var springForce = -p.contactSpring.ks * Vector3.Dot(p.position - p.contactSpring.attachPoint, plane.normal) * plane.normal;
                // damper
                springForce += -p.contactSpring.kd * p.velocity;
                p.currentForces += springForce;
            }
        }

        //spring
        foreach (var spring in p.attachedSprings)
        {
            var otherP = particles[spring.attachedParticle];

            if ((p.position - otherP.position).magnitude == 0.0f) continue; // this was causing issues

            //spring force
            var springForce = spring.ks * (spring.restLength - (p.position - otherP.position).magnitude) * (p.position - otherP.position) / (p.position - otherP.position).magnitude;
            // damper
            springForce += -spring.kd * Vector3.Dot(p.velocity - otherP.velocity, (p.position - otherP.position) / (p.position - otherP.position).magnitude) * (p.position - otherP.position) / (p.position - otherP.position).magnitude;
            p.currentForces += springForce;
            otherP.currentForces += -springForce;
            particles[spring.attachedParticle] = otherP;
        }
        
        particles[index] = p;
    }

    void ResetParticleForces()
    {
        for (var i = 0; i < particles.Length;i++)
        {
            var p = particles[i];
            p.currentForces = Vector3.zero;
            particles[i] = p;
        }
    }


    void FixedUpdate()
    {
        ResetParticleForces();
        SimulationStep();
        UpdateMesh();
    }


    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
        // This will work if you have a correctly made particles array
        if (debugRender)
        {
            int particleCount = particles.Length;
            for (int i = 0; i < particleCount; i++)
            {
                Debug.DrawLine(particles[i].position, particles[i].position + particles[i].currentForces, Color.blue, 0, true);

                int springCount = particles[i].attachedSprings.Count;
                for (int j = 0; j < springCount; j++)
                {
                    Debug.DrawLine(particles[i].position, particles[particles[i].attachedSprings[j].attachedParticle].position, Color.red, 0, true);
                }
            }
        }
    }
}
