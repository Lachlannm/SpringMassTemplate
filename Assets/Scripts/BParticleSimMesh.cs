using System.Collections;
using System.Collections.Generic;
using System.Linq;
using Unity.VisualScripting;
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
    public bool handlePlaneCollisions;
    public float mass;
    public bool useGravity;
    public Vector3 gravity;

    private List<BParticle> particles = new();
    private Mesh mesh;
    private BPlane plane;

    

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
        // extract particles from the mesh
        mesh = GetComponent<MeshFilter>().mesh;
        var vertices = mesh.vertices.GroupBy(p => p);
        print(vertices.Count());

        // extract vertices from mesh without duplication
        foreach (var vertex in vertices)
        {
            var v = vertex.FirstOrDefault();
            var particle = new BParticle();
            particle.position = transform.TransformPoint(v);
            particle.velocity = Vector3.zero;
            particle.mass = mass;
            particle.contactSpring = new BContactSpring();
            particle.attachedToContact = false;
            particle.attachedSprings = new List<BSpring>();
            particle.currentForces = Vector3.zero;
            particles.Add(particle);
            print(particle.position);
        }

        // set up the springs
        foreach (var p1 in particles)
        {
            foreach (var p2 in particles)
            {
                // don't attach a spring between the same particle
                if (p1.position == p2.position) continue;

                //check if a spring already exists for this connection
                if (p2.attachedSprings.Exists(s => s.attachedParticle == particles.IndexOf(p1))) continue;

                //add spring
                var spring = new BSpring();
                spring.attachedParticle = particles.IndexOf(p2);
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
        
    }

    void SimulationStep()
    {
        var delta = Time.deltaTime;
        for (var i = 0; i < particles.Count();i++)
        {
            var p = particles[i];
            p.position.x += 0.1f;
            particles[i] = p;
            
        }
        UpdateMesh();
        ResetParticleForces();
    }
    
    void ResetParticleForces()
    {
        for (var i = 0; i < particles.Count();i++)
        {
            var p = particles[i];
            p.currentForces = Vector3.zero;
            particles[i] = p;
        }
    }


    /*** BIG HINT: My solution code has as least the following functions
     * InitParticles()
     * InitPlane()
     * UpdateMesh() (remember the hint above regarding global and local coords)
     * ResetParticleForces()
     * ...
     ***/

    void FixedUpdate()
    {
        SimulationStep();
    }


    /// <summary>
    /// Draw a frame with some helper debug render code
    /// </summary>
    public void Update()
    {
        // This will work if you have a correctly made particles array
        if (debugRender)
        {
            int particleCount = particles.Count;
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
