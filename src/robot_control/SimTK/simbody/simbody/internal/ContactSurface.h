#ifndef SimTK_SIMBODY_CONTACT_SURFACE_H_
#define SimTK_SIMBODY_CONTACT_SURFACE_H_

/* -------------------------------------------------------------------------- *
 *                               Simbody(tm)                                  *
 * -------------------------------------------------------------------------- *
 * This is part of the SimTK biosimulation toolkit originating from           *
 * Simbios, the NIH National Center for Physics-Based Simulation of           *
 * Biological Structures at Stanford, funded under the NIH Roadmap for        *
 * Medical Research, grant U54 GM072970. See https://simtk.org/home/simbody.  *
 *                                                                            *
 * Portions copyright (c) 2008-13 Stanford University and the Authors.        *
 * Authors: Peter Eastman, Michael Sherman                                    *
 * Contributors:                                                              *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

/** @file 
Declares ContactMaterial and ContactSurface classes. **/

#include "SimTKmath.h"
#include "simbody/internal/common.h"

#include <algorithm>

namespace SimTK {

class ContactGeometry;

SimTK_DEFINE_UNIQUE_INDEX_TYPE(ContactCliqueId);



//==============================================================================
//                               CONTACT MATERIAL
//==============================================================================
/** Define the physical properties of the material from which a contact
surface is made, including properties needed by a variety of contact response
techniques that might be applied during contact. Two basic techniques are
supported:
  - compliant contact
  - rigid contact

Each of these techniques can respond to both impacts and continuous contact,
and provides for both compressive and frictional effects.

For compliant contact this material can be applied in different ways:
  - the material represents an elastic solid, large in comparison to the 
    region of contact deformation
  - the material represents a uniform thin elastic layer over an 
    otherwise-rigid body (elastic foundation model).

The elastic foundation layer thickness h is \e not a property of this material;
instead it is specified at the time this material is used to construct a 
ContactSurface; the material properties supplied here are independent of how
the material is used.

<h3>Compliant vs. rigid contact models</h3>
  
Compliant contact models respond via forces that are generated by 
finite deformations of the contact surface, using theory of elasticity. Rigid 
contact models instead enforce zero deformation at the contact surface via 
the reaction forces of non-penetration contraints, and respond to impacts by 
generating impulsive changes to velocities rather than forces.

Energy dissipation upon impact is dealt with differently in the two schemes. 
In compliant contact, energy dissipation occurs via a continuous function
of deformation and deformation rate, based on a material property c we call
the "dissipation coefficient" (Hunt & Crossley call this "alpha"). In rigid 
contact, energy dissipation occurs via a "coefficient of restitution" e that 
defines what fraction of (a) impact velocity (Newton's hypothesis) or (b)
collision impulse (Poisson's hypothesis) is lost on rebound. It is common 
practice, but blatantly incorrect, to use a constant value for the coefficient 
of restitution. Empirically, coefficient of restitution e is known to be a 
linear function of impact velocity for small velocities: e=1-c*v where c is 
the dissipation coefficient mentioned above. At large velocities plasticity and
other effects cause the coefficient of restitution to drop dramatically; we 
do not attempt to support plasticity models here. **/
class SimTK_SIMBODY_EXPORT ContactMaterial {
public:
/** Default constructor creates an invalid contact material; you must 
specify at least stiffness before using this material. Defaults are provided
for dissipation (none) and friction (none). **/
ContactMaterial() {clear();}

/** Create a contact material with a complete set of compliant contact
material properties. This does not set the coefficient of restitution. 
There are a variety of static methods provided to aid in calculating the
stiffness and dissipation from available data. 
@param[in]      stiffness
    This is in units of pressure per unit strain, where pressure is force per
    unit area, and strain is a unitless measure of deformation. There are
    a variety of ways to compute this from Young's modulus and Poisson's
    ratio; see for example calcPlaneStrainStiffness() and 
    calcConfinedCompressionStiffness(). Note that different compliant contact 
    models determine strain differently for a given deformation x. For example,
    Hertz uses the relationship between x and local curvatures to determine the
    fractional strain, while Elastic Foundation must be given a "thickness" h 
    with strain then defined as x/h. We require \a stiffness >= 0.
@param[in]      dissipation
    This is the Hunt and Crossley dissipation coefficient for this material
    in units of 1/velocity. This is the fraction of stiffness force generated
    as a dissipation force per unit deformation rate, via
    f_dissipation = f_stiffness * (dissipation * v_deformation). You can
    measure this as the slope of the coefficient of restitution (e) vs. impact
    velocity (v) curve at low velocities, where e=1-c*v.
    We require \a dissipation >= 0.
@param[in]      staticFriction
    This is the Coulomb "stiction" coefficient mu_s that is used at very low
    sliding velocities, with tangential force limited to f=mu_s*N where N
    is the contact normal force due to stiffness and dissipation effects. 
    We require \a staticFriction >= \a dynamicFriction >= 0.
@param[in]      dynamicFriction
    This is the Coulomb friction coefficient mu_d that is to be used at 
    significant sliding velocities, with velocity-independent tangential 
    force magnitude f=mu_d*N produced opposing the sliding direction, where 
    N is the contact normal force due to stiffness and dissipation effects. 
    We require 0 <= \a dynamicFriction <= \a staticFriction.
@param[in]      viscousFriction
    This is the "wet" friction coefficient mu_v >= 0 for this material 
    generating a sliding velocity-dependent tangential force of magnitude 
    f=mu_v*v*N where v is the sliding velocity and N is the contact normal
    force due to stiffness and dissipation effects. This is not commonly
    used and defaults to zero.
**/
ContactMaterial(Real stiffness, Real dissipation, 
                Real staticFriction, Real dynamicFriction,
                Real viscousFriction = 0) {
    setStiffness(stiffness);
    setDissipation(dissipation);
    setFriction(staticFriction, dynamicFriction, viscousFriction);
}

/** Return false if material properties have not yet been supplied for this
contact material. **/
bool isValid() const {return m_stiffness > 0;}

/** Return the material stiffness k=(force/area)/%%strain. **/
Real getStiffness() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getStiffness()",
        "This is an invalid ContactMaterial.");
    return m_stiffness; }
/** Return precalculated 2/3 power of material stiffness k (k^(2/3)). **/
Real getStiffness23() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getStiffness23()",
        "This is an invalid ContactMaterial.");
    return m_stiffness23; }
/** Return the material dissipation coefficient c, in units of 1/velocity. **/
Real getDissipation() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getDissipation()",
        "This is an invalid ContactMaterial.");
    return m_dissipation; }
/** Return the coefficient of static friction mu_s (unitless). **/
Real getStaticFriction() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getStaticFriction()",
        "This is an invalid ContactMaterial.");
    return m_staticFriction; }
/** Return the coefficient of dynamic friction mu_d (unitless). **/
Real getDynamicFriction() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getDynamicFriction()",
        "This is an invalid ContactMaterial.");
    return m_dynamicFriction; }
/** Return the coefficient of viscous friction mu_v (1/velocity). **/
Real getViscousFriction() const 
{   SimTK_ERRCHK(isValid(), "ContactMaterial::getViscousFriction()",
        "This is an invalid ContactMaterial.");
    return m_viscousFriction; }

/** Supply material stiffness k=(force/area)/%%strain. A compliant
contact model will then calculate the contact area A and strain fraction
s to calculate the resulting force f=k*A*s. **/
ContactMaterial& setStiffness(Real stiffness) {
    SimTK_ERRCHK1_ALWAYS(stiffness >= 0, "ContactMaterial::setStiffness()",
        "Stiffness %g is illegal; must be >= 0.", stiffness);
    m_stiffness = stiffness;
    m_stiffness23 = std::pow(m_stiffness, Real(2./3.));
    return *this;
}

/** Supply material dissipation coefficient c, which is the slope of the
coefficient of restitution (e) vs. impact speed (v) curve, with e=1-cv at
low (non-yielding) impact speeds v. **/
ContactMaterial& setDissipation(Real dissipation) {
    SimTK_ERRCHK1_ALWAYS(dissipation >= 0, "ContactMaterial::setDissipation()",
        "Dissipation %g (in 1/speed) is illegal; must be >= 0.", dissipation);
    m_dissipation = dissipation;
    return *this;
}

/** Set the friction coefficients for this material. **/
ContactMaterial& setFriction(Real staticFriction,
                             Real dynamicFriction,
                             Real viscousFriction = 0)
{
    SimTK_ERRCHK1_ALWAYS(0 <= staticFriction, "ContactMaterial::setFriction()",
        "Illegal static friction coefficient %g.", staticFriction);
    SimTK_ERRCHK2_ALWAYS(0<=dynamicFriction && dynamicFriction<=staticFriction, 
        "ContactMaterial::setFriction()",
        "Dynamic coefficient %g illegal; must be between 0 and static"
        " coefficient %g.", dynamicFriction, staticFriction);
    SimTK_ERRCHK1_ALWAYS(0 <= viscousFriction, "ContactMaterial::setFriction()",
        "Illegal viscous friction coefficient %g.", viscousFriction);

    m_staticFriction  = staticFriction;
    m_dynamicFriction = dynamicFriction;
    m_viscousFriction = viscousFriction;
    return *this;
}

/** Calculate the contact material stiffness k from its linear elastic 
material properties Young's modulus E and Poisson's ratio v, assuming that
the material is free to move under pressure. In this circumstance the
effective stiffness is given by the plane strain modulus k=E/(1-v^2), and
the stiffness never exceeds (4/3)E when the material becomes incompressible
as v->0.5 as is typical of rubber. If this material is to be used in a 
circumstance in which its volume must be reduced under contact pressure (because
there is no room to move), consider using the confined compression modulus
instead. @see calcConfinedCompressionStiffness() **/
static Real calcPlaneStrainStiffness(Real youngsModulus, 
                                     Real poissonsRatio) 
{
    SimTK_ERRCHK2_ALWAYS(youngsModulus >= 0 &&
                         -1 < poissonsRatio && poissonsRatio <= 0.5,
                         "ContactMaterial::calcStiffnessForSolid()",
        "Illegal material properties E=%g, v=%g.", 
        youngsModulus, poissonsRatio);

    return youngsModulus / (1-square(poissonsRatio)); 
}


/** Calculate the contact material stiffness k from its linear elastic 
material properties Young's modulus E and Poisson's ratio v, assuming that
the material is confined such that its volume must be reduced under pressure.
In this case the effective stiffness is given by the confined compression
modulus k=E(1-v)/((1+v)(1-2v)). Note that the stiffness becomes infinite when
the material becomes incompressible as v->0.5. That means rubber will have
a near infinite stiffness if you calculate it this way; make sure that's what
you want! @see calcPlaneStrainStiffness() **/
static Real calcConfinedCompressionStiffness(Real youngsModulus, 
                                             Real poissonsRatio) 
{
    SimTK_ERRCHK2_ALWAYS(youngsModulus >= 0 &&
                         -1 < poissonsRatio && poissonsRatio < 0.5,
                         "ContactMaterial::calcStiffnessForSolid()",
        "Illegal material properties E=%g, v=%g.", 
        youngsModulus, poissonsRatio);

    return youngsModulus*(1-poissonsRatio) 
        / ((1+poissonsRatio)*(1-2*poissonsRatio)); 
}

/** Given an observation of the coefficient of restitution e at a given small 
impact velocity v, calculate the apparent value of the dissipation coefficient
c, which is a material property. You should get the same value for c from
any observation, provided that v is non-zero but small. The coefficient of
restitution as v->0 is 1 for all materials.

Coefficient of restitution at low impact speed is linearly related to 
impact velocity by the dissipation coefficient c (1/velocity) by e=(1-cv) so we
can calculate c if we have observed e at some low speed v as c=(1-e)/v. If the
coefficient of restitution is 1 we'll return c=0 and ignore v, otherwise
v must be greater than zero. **/
static Real calcDissipationFromObservedRestitution
   (Real restitution, Real speed) {
    if (restitution==1) return 0;
    SimTK_ERRCHK2_ALWAYS(0<=restitution && restitution<=1 && speed>0,
        "ContactMaterial::calcDissipationFromRestitution()",
        "Illegal coefficient of restitution or speed (%g,%g).",
        restitution, speed);
    return (1-restitution)/speed;
}

/** Restore this contact material to an invalid state containing no stiffness
specification and default values for dissipation (none) and friction 
(none). **/
void clear() {
    m_stiffness   = NaN; // unspecified
    m_stiffness23 = NaN;
    m_restitution = NaN; // unspecified
    m_dissipation = 0;  // default; no dissipation
    // default; no friction
    m_staticFriction=m_dynamicFriction=m_viscousFriction = 0;
}

//--------------------------------------------------------------------------
private:

// For compliant contact models.
Real    m_stiffness;        // k: stress/%strain=(force/area)/%strain
Real    m_stiffness23;      // k^(2/3) in case we need it
Real    m_dissipation;      // c: %normalForce/normalVelocity

// For impulsive collisions.
Real    m_restitution;      // e: unitless, e=(1-cv)

// Friction.
Real    m_staticFriction;   // us: unitless
Real    m_dynamicFriction;  // ud: unitless
Real    m_viscousFriction;  // uv: %normalForce/slipVelocity
};



//==============================================================================
//                               CONTACT SURFACE
//==============================================================================
/** This class combines a piece of ContactGeometry with a ContactMaterial to
make an object suitable for attaching to a body which can then engage in
contact behavior with other contact surfaces. The ContactMaterial may be
considered as uniform throughout a large solid volume or as a thin layer of 
uniform material thickness h on top of a rigid substrate. Compliant contact
models vary in how they deal with the thickness h: Hertz ignores it, Elastic
Foundation uses it to define unit strain.

Typically any contact surface can bump into any other contact surface. However, 
some groups of surfaces can or should never interact; those groups are called 
"contact cliques". The set of surfaces that are mounted to the same rigid body 
always constitues a clique. In addition, other cliques may be defined and 
arbitrary surfaces made members so that they won't interact. This is commonly 
used for contact surfaces on adjacent bodies when those surfaces are near the 
joint between two bodies. **/ 
class SimTK_SIMBODY_EXPORT ContactSurface {
public:
/** Create an empty ContactSurface. **/
ContactSurface() {}
/** Create a ContactSurface with a given shape and material. **/
ContactSurface(const ContactGeometry&   shape, 
               const ContactMaterial&   material,
               Real                     thickness=0)
:   m_shape(shape), m_material(material), m_thickness(thickness) {
    SimTK_ERRCHK1_ALWAYS(thickness >= 0, "ContactSurface::ctor()",
        "Illegal thickness %g.", thickness);
}

/** Define a new shape for this ContactSurface. **/
ContactSurface& setShape(const ContactGeometry& shape) 
{   m_shape = shape; return *this; }

/** Define a new material for this ContactSurface, optionally providing
the thickness of this elastic material layer over a rigid substrate. **/
ContactSurface& setMaterial(const ContactMaterial& material,
                            Real thickness = 0) 
{   m_material = material; setThickness(thickness); return *this; }

/** Set the thickness of the layer of elastic material coating this contact
surface. This is required by the Elastic Foundation Model but is ignored
by the Hertz model. Elastic Foundation uses this value to define "unit strain",
that is, an element deformed by 0.1*thickness has 10% strain. **/
ContactSurface& setThickness(Real thickness) {
    SimTK_ERRCHK1_ALWAYS(thickness >= 0, "ContactSurface::setThickness()",
        "Illegal thickness %g.", thickness);
    m_thickness = thickness; 
    return *this; 
}

/** Get read-only access to the shape of this contact surface. **/
const ContactGeometry& getShape()    const {return m_shape;}

/** Get read-only access to the material of this contact surface. **/
const ContactMaterial& getMaterial() const {return m_material;}

/** Get the thickness of the elastic material layer on this contact surface,
with zero indicating that the thickness has not been set. **/
Real getThickness() const {return m_thickness;}

/** Get writable access to the shape of this contact surface. **/
ContactGeometry& updShape()    {return m_shape;}

/** Get writable access to the material of this contact surface. **/
ContactMaterial& updMaterial() {return m_material;}

/** Join a contact clique if not already a member. It is OK to pass an invalid
clique id here, in which case it will be quietly ignored. **/
ContactSurface& joinClique(ContactCliqueId clique) {
    if (!clique.isValid()) return *this;
    // Although this is a sorted list, we expect it to be very short so 
    // are using linear search to find where this new clique goes.
    Array_<ContactCliqueId,short>::iterator p;
    for (p = m_cliques.begin(); p != m_cliques.end(); ++p) {   
        if (*p==clique) return *this; // already a member
        if (*p>clique) break;
    }
    // insert just before p (might be at the end)
    m_cliques.insert(p, clique);
    return *this;
}

/** Remove this surface from a contact clique if it is a member. It is OK to 
pass an invalid clique id here, in which case it will be quietly ignored. **/
void leaveClique(ContactCliqueId clique) {   
    if (!clique.isValid()) return;
    // We expect this to be a very short list so are using linear search.
    Array_<ContactCliqueId,short>::iterator p =
        std::find(m_cliques.begin(), m_cliques.end(), clique);
    if (p != m_cliques.end()) m_cliques.erase(p); 
}

/** Determine whether this contact surface is a member of any of the same
cliques as some \a other contact surface, in which case they will be invisible
to one another. **/
bool isInSameClique(const ContactSurface& other) const 
{   if (getCliques().empty() || other.getCliques().empty()) return false;//typical
    return cliquesIntersect(getCliques(), other.getCliques()); }

/** Return a reference to the list of CliqueIds of the cliques in which this
contact surface is a member; the list is always sorted in ascending order. **/
const Array_<ContactCliqueId,short>& getCliques() const {return m_cliques;}

/** Return true if there are any common cliques on two clique lists which
must each be sorted in ascending order. This takes no longer than O(a+b)
where a and b are the sizes of the two clique lists. **/
static bool cliquesIntersect(const Array_<ContactCliqueId,short>& a,
                             const Array_<ContactCliqueId,short>& b)
{
    Array_<ContactCliqueId,short>::const_iterator ap = a.begin();
    Array_<ContactCliqueId,short>::const_iterator bp = b.begin();
    // Quick checks: empty or non-overlapping.
    if (ap==a.end() || bp==b.end()) return false;
    if (*ap > b.back() || *bp > a.back()) 
        return false; // disjoint
    // Both lists have elements and the elements overlap numerically.
    while (true) {
        if (*ap==*bp) return true;
        // Increment the list with smaller front element.
        if (*ap < *bp) {++ap; if (ap==a.end()) break;}
        else           {++bp; if (bp==b.end()) break;}
    }
    // One of the lists ran out of elements before we found a match.
    return false;
}

/** Create a new contact clique and return its unique integer id (thread
safe). Every contact surface is automatically a member of a clique containing 
all the surfaces that reside on the same body. **/
static ContactCliqueId createNewContactClique()
{   static AtomicInteger nextAvailableContactClique = 1;
    return ContactCliqueId(nextAvailableContactClique++); }

//----------------------------------------------------------------------
                                 private:

ContactGeometry                 m_shape;
ContactMaterial                 m_material;
Real                            m_thickness; // default=Infinity
Array_<ContactCliqueId,short>   m_cliques;   // sorted
};



} // namespace SimTK

#endif // SimTK_SIMBODY_CONTACT_SURFACE_H_
