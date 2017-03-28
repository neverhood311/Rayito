////////////////////////////////////////////////////////////////////////////////
//
// Very simple ray tracing example
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __RMATERIAL_H__
#define __RMATERIAL_H__

#include "RMath.h"
#include "RRay.h"
#include "RSampling.h"


namespace Rayito
{

//
// BSDF (bidirectional scattering distribution function)
//

class Bsdf
{
public:
    Bsdf() {}

    virtual ~Bsdf() {}

    /*
     * BSDF details:
     *
     * There are two variations of each of these functions: those that produce
     * PDFs with respect to solid angle (not accounting for angle between
     * outgoing vector and the normal) or projected solid angle (the opposite).
     * The PDFs always go from 0.0 to 1.0.
     *
     * These BSDFs must also be energy-conserving, meaning they cannot ever
     * reflect back more light than they receive when you gather all of the light
     * reflected over the entire hemisphere around the surface normal.
     *
     * The evaluate methods return the reflectance given the incoming and
     * outgoing directions and the surface normal, and also output the PDF
     * (probability distribution function) value for that configuration of
     * in/out/normal.
     *
     * The sample methods will generate an incoming direction given an outgoing
     * direction and normal, also returning the PDF value for having generated
     * that incoming direction.
     *
     * The pdf methods just return the PDF of the in/out/normal configuration.
     *
     * The Dirac distribution flag indicates this is a BSDF that really only
     * reflects in one direction (perfect reflection or refraction), indicating
     * the PDF of a given in/out/normal combo will always be zero.  However,
     * the sample methods will return the exact input direction with a PDF of 1.0.
    */

    virtual float evaluateSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf)  const = 0;
    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        float scattering = evaluateSA(incoming, outgoing, normal, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(incoming, normal));
        return scattering;
    }

    virtual float sampleSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf)  const = 0;
    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        float scattering = sampleSA(outIncoming, outgoing, normal, u1, u2, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(outIncoming, normal));
        return scattering;
    }

    virtual float pdfSA(const Vector& incoming, const Vector& outgoing, const Vector& normal)  const = 0;
    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // Convert from solid-angle PDF to projected-solid-angle PDF
        return pdfSA(incoming, outgoing, normal) / std::fabs(dot(incoming, normal));
    }

    virtual bool isDiracDistribution() const { return false; }
};

//
// BRDF (bidirectional reflectance distribution function)
//

class Brdf : public Bsdf
{
public:
    Brdf() { }
    
    virtual ~Brdf() { }
    
    /*
     * BRDF details:
     * 
     * Same as BSDF with the following addition:
     * The outgoing direction is away from the surface in the same hemisphere as
     * the normal, while the incoming is toward the surface opposite the normal.
     *
     */
    
    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        float reflectance = evaluateSA(incoming, outgoing, normal, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(incoming, normal));
        return reflectance;
    }
    
    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        float reflectance = sampleSA(outIncoming, outgoing, normal, u1, u2, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(outIncoming, normal));
        return reflectance;
    }
    
    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // Convert from solid-angle PDF to projected-solid-angle PDF
        return pdfSA(incoming, outgoing, normal) / std::fabs(dot(incoming, normal));
    }
    
    virtual bool isDiracDistribution() const { return false; }
};

//
// Bidirectional Transmittance Distribution Function
//

class Btdf : public Bsdf
{
public:
    Btdf() { }

    virtual ~Btdf() { }

    /*
     * BTDF details:
     *
     * Same as BSDF with the following addition:
     * The outgoing direction is away from the surface in the opposite hemisphere as
     * the normal, while the incoming is toward the surface in the same direction as the normal.
     *
     */

    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        float transmittance = evaluateSA(incoming, outgoing, normal, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(incoming, normal));
        return transmittance;
    }

    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        float transmittance = sampleSA(outIncoming, outgoing, normal, u1, u2, outPdf);
        // Convert from solid-angle PDF to projected-solid-angle PDF
        outPdf /= std::fabs(dot(outIncoming, normal));
        return transmittance;
    }

    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // Convert from solid-angle PDF to projected-solid-angle PDF
        return pdfSA(incoming, outgoing, normal) / std::fabs(dot(incoming, normal));
    }

    virtual bool isDiracDistribution() const { return false; }
};


// Lambertian diffuse BRDF
class Lambert : public Brdf
{
public:
    Lambert() : Brdf() { }
    
    virtual ~Lambert() { }
    
    
    virtual float evaluateSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            outPdf = 0.0f;
            return 0.0f;
        }
        // Standard solid-angle PDF for diffuse reflectance
        outPdf = std::fabs(nDotI) / M_PI;
        return 1.0f / M_PI;
    }
    
    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            outPdf = 0.0f;
            return 0.0f;
        }
        // Standard projected-solid-angle PDF for diffuse reflectance;
        // and yes, Virginia, they're the same (and end up canceling out later)
        outPdf = 1.0f / M_PI;
        return 1.0f / M_PI;
    }
    
    
    virtual float sampleSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        // Generate incoming direction towards normal in the hemisphere,
        // favoring directions near the pole
        Vector localIncoming = -uniformToCosineHemisphere(u1, u2);
        // Transform incoming direction to where the pole lines up with the normal
        Vector x, y, z;
        makeCoordinateSpace(normal, x, y, z);
        outIncoming = transformFromLocalCoordinateSpace(localIncoming, x, y, z);
        // If the outgoing direction is below the normal's hemisphere,
        // flip the generated incoming direction as well
        if (dot(outgoing, normal) < 0.0f)
            outIncoming *= -1.0f;
        // Standard solid-angle PDF for diffuse reflectance
        outPdf = std::fabs(dot(-outIncoming, normal)) / M_PI;
        // Normalized diffuse reflectance is always 1/pi
        return 1.0f / M_PI;
    }
    
    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        // Generate incoming direction towards normal in the hemisphere,
        // favoring directions near the pole
        Vector localIncoming = -uniformToCosineHemisphere(u1, u2);
        // Transform incoming direction to where the pole lines up with the normal
        Vector x, y, z;
        makeCoordinateSpace(normal, x, y, z);
        outIncoming = transformFromLocalCoordinateSpace(localIncoming, x, y, z);
        // If the outgoing direction is below the normal's hemisphere,
        // flip the generated incoming direction as well
        if (dot(outgoing, normal) < 0.0f)
            outIncoming *= -1.0f;
        // Standard projected-solid-angle PDF for diffuse reflectance;
        // and yes, Virginia, they're the same (and end up canceling out later)
        outPdf = 1.0f / M_PI;
        // Normalized diffuse reflectance is always 1/pi
        return 1.0f / M_PI;
    }
    
    
    virtual float pdfSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }
        // Standard solid-angle PDF for diffuse reflectance
        return std::fabs(nDotI) / M_PI;
    }
    
    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }
        // Standard projected-solid-angle PDF for diffuse reflectance
        return 1.0f / M_PI;
    }
};

// Oren-Nayar diffuse BRDF
class OrenNayar : public Brdf
{
public:
    OrenNayar() : Brdf() {
        float sig = 1.0f;
        float sig2 = sig * sig;
        m_A = 1.0f - (sig2 / (2.0f * (sig2 + 0.33f)));
        m_B = 0.45f * sig2 / (sig2 + 0.09f);
    }

    OrenNayar(float sigma) : Brdf() {
        float sig = sigma;
        float sig2 = sig * sig;
        m_A = 1.0f - (sig2 / (2.0f * (sig2 + 0.33f)));
        m_B = 0.45f * sig2 / (sig2 + 0.09f);
    }

    virtual ~OrenNayar() { }

    virtual float evaluateSA(const Vector &incoming, const Vector &outgoing, const Vector &normal, float &outPdf) const
    {
        float retval = evaluatePSA(incoming, outgoing, normal, outPdf);
        outPdf *= std::fabs(dot(incoming, normal));
        return retval;
    }

    virtual float evaluatePSA(const Vector &incoming, const Vector &outgoing, const Vector &normal, float &outPdf) const
    {
        //PBRT 2nd edition, page 451
        float sinthetai = SinTheta(incoming);
        float sinthetao = SinTheta(outgoing);   //TODO: make sure these vectors are in the same space as those in PBRT

        float maxcos = 0.0f;
        if(sinthetai > 1e-4 && sinthetao > 1e-4){
            float sinphii = SinPhi(incoming), cosphii = CosPhi(incoming);
            float sinphio = SinPhi(outgoing), cosphio = CosPhi(outgoing);
            float dcos = cosphii * cosphio + sinphii * sinphio;
            maxcos = std::max(0.0f, dcos);
        }

        float sinalpha, tanbeta;
        if(AbsCosTheta(incoming) > AbsCosTheta(outgoing)){
            sinalpha = sinthetao;
            tanbeta = sinthetai / AbsCosTheta(incoming);
        }
        else{
            sinalpha = sinthetai;
            tanbeta = sinthetao / AbsCosTheta(outgoing);
        }

        outPdf = INV_PI;    //PBRT 2nd edition, page 695 says to do this
        return INV_PI * (m_A + m_B * maxcos * sinalpha * tanbeta);
    }

    virtual float sampleSA(Vector &outIncoming, const Vector &outgoing, const Vector &normal, float u1, float u2, float &outPdf) const
    {
        float retval = samplePSA(outIncoming, outgoing, normal, u1, u2, outPdf);
        outPdf *= std::fabs(dot(outIncoming, normal));
        return retval;
    }

    virtual float samplePSA(Vector &outIncoming, const Vector &outgoing, const Vector &normal, float u1, float u2, float &outPdf) const
    {
        // Pick a direction in the hemisphere (eventually stored in outIncoming)
        // http://research.cs.wisc.edu/graphics/Courses/779-s2005/lectures/cs779-7.ppt says we should do cosine-weighted hemisphere
        // Also PBRT 2nd edition, page 694 says to do the cosine-weighted hemisphere
        Vector localIncoming = -uniformToCosineHemisphere(u1, u2);
        // Transform the incoming direction to where the pole lines up with the normal
        Vector x, y, z;
        makeCoordinateSpace(normal, x, y, z);
        outIncoming = transformFromLocalCoordinateSpace(localIncoming, x, y, z);
        // If the outgoing direction is below the normal's hemisphere,
        // flip the generated incoming direction also
        if (dot(outgoing, normal) < 0.0f)
            outIncoming *= -1.0f;
        // Calculate the projected solid angle pdf for Oren-Nayar
        // (ask the brdf what it thinks about this direction)
        return evaluatePSA(outIncoming, outgoing, normal, outPdf);
    }

    virtual float pdfSA(const Vector &incoming, const Vector &outgoing, const Vector &normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }

        return pdfPSA(incoming, outgoing, normal) * std::fabs(nDotI);
    }

    virtual float pdfPSA(const Vector &incoming, const Vector &outgoing, const Vector &normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }

        float outPdf = 0.0f;
        evaluatePSA(incoming, outgoing, normal, outPdf);
        return outPdf;
    }

protected:
    float m_A, m_B;
};

// Ashikhmin-Shirley glossy BRDF, without anisotropy
class Glossy : public Brdf
{
public:
    Glossy(float roughness) : Brdf(), m_exponent(1.0f / (roughness * roughness)) { }
    
    virtual ~Glossy() { }
    
    
    // Note used yet, but it may be in the future (the standard A-S model has this)
    float schlickFresnel(float reflectionIncidentToNormal,
                         float cosTheta) const
    {
        // This is a cheaper-to-compute approximation of a true dielectric Fresnel
        return reflectionIncidentToNormal +
                (1.0f - reflectionIncidentToNormal) *
                std::pow(std::max(1.0f - cosTheta, 0.0f), 5);
    }
    
    virtual float evaluateSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            outPdf = 0.0f;
            return 0.0f;
        }
        // Compute microfacet model half-vector, taking care not to produce garbage
        Vector half;
        if (dot(outgoing, incoming) > 0.999f)
            half = normal;
        else
            half = (outgoing - incoming).normalized();
        float fresnel = 1.0f; // TODO: add Fresnel
        // Standard A-S model components, but using the D-BRDF microfacet denominator
        float d = (m_exponent + 1.0f) * std::pow(std::fabs(dot(normal, half)), m_exponent) / (2.0f * M_PI);
        float result = fresnel * d / (4.0f * std::fabs(nDotO + -nDotI - nDotO * -nDotI));
        outPdf = d / (4.0f * std::fabs(dot(outgoing, half)));
        return result;
    }
    
    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            outPdf = 0.0f;
            return 0.0f;
        }
        // Compute microfacet model half-vector, taking care not to produce garbage
        Vector half;
        if (dot(outgoing, incoming) > 0.999f)
            half = normal;
        else
            half = (outgoing - incoming).normalized();
        float fresnel = 1.0f; // TODO: add Fresnel
        // Standard A-S model components, but using the D-BRDF microfacet denominator
        float d = (m_exponent + 1.0f) * std::pow(std::fabs(dot(normal, half)), m_exponent) / (2.0f * M_PI);
        float result = fresnel * d / (4.0f * std::fabs(nDotO + -nDotI - nDotO * -nDotI));
        outPdf = d / (4.0f * std::fabs(dot(outgoing, half)) * std::fabs(nDotI));
        return result;
    }
    
    
    virtual float sampleSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        // Generate half-vector from the A-S model
        float phi = 2.0f * M_PI * u1;
        float cosTheta = std::pow(1.0f - u2, 1.0f / (m_exponent + 1.0f));
        float sin2Theta = std::max(0.0f, 1.0f - cosTheta * cosTheta);
        float sinTheta = std::sqrt(sin2Theta);
        Vector localHalf(sinTheta * std::cos(phi),
                         sinTheta * std::sin(phi),
                         cosTheta);
        // Transform half-vector to be based around the world-space normal
        Vector x, y, z;
        makeCoordinateSpace(normal, x, y, z);
        Vector half = transformFromLocalCoordinateSpace(localHalf, x, y, z);
        // If outgoing is on the other side of the normal, flip the half-vector too
        if (dot(outgoing, normal) < 0.0f)
            half *= -1.0f;
        // Reflect outgoing past half-vector to get incoming
        outIncoming = outgoing - half * (2.0f * dot(outgoing, half));
        // Evaluate the A-S model to get final reflectance and PDF
        return evaluateSA(outIncoming, outgoing, normal, outPdf);
    }
    
    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        // Generate half-vector from the A-S model
        float phi = 2.0f * M_PI * u1;
        float cosTheta = std::pow(1.0f - u2, 1.0f / (m_exponent + 1.0f));
        float sin2Theta = std::max(0.0f, 1.0f - cosTheta * cosTheta);
        float sinTheta = std::sqrt(sin2Theta);
        Vector localHalf(sinTheta * std::cos(phi),
                         sinTheta * std::sin(phi),
                         cosTheta);
        // Transform half-vector to be based around the world-space normal
        Vector x, y, z;
        makeCoordinateSpace(normal, x, y, z);
        Vector half = transformFromLocalCoordinateSpace(localHalf, x, y, z);
        // If outgoing is on the other side of the normal, flip the half-vector too
        if (dot(outgoing, normal) < 0.0f)
            half *= -1.0f;
        // Reflect outgoing past half-vector to get incoming
        outIncoming = outgoing - half * (2.0f * dot(outgoing, half));
        // Evaluate the A-S model to get final reflectance and PDF
        return evaluatePSA(outIncoming, outgoing, normal, outPdf);
    }
    
    
    virtual float pdfSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }
        // Compute microfacet model half-vector, taking care not to produce garbage
        Vector half;
        if (dot(outgoing, incoming) > 0.999f)
            half = normal;
        else
            half = (outgoing - incoming).normalized();
        // Evaluate the A-S model to get final PDF
        return (m_exponent + 1.0f) * std::pow(std::fabs(dot(normal, half)), m_exponent) /
               (8.0f * M_PI * std::fabs(dot(outgoing, half)));
    }
    
    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        // If incoming and outgoing are in the same hemisphere, that means the
        // direction is on the backside of the other and no reflection occurs
        float nDotI = dot(incoming, normal);
        float nDotO = dot(outgoing, normal);
        if ((nDotI > 0.0f && nDotO > 0.0f) ||
            (nDotI < 0.0f && nDotO < 0.0f))
        {
            return 0.0f;
        }
        // Compute microfacet model half-vector, taking care not to produce garbage
        Vector half;
        if (dot(outgoing, incoming) > 0.999f)
            half = normal;
        else
            half = (outgoing - incoming).normalized();
        // Evaluate the A-S model to get final PDF
        return (m_exponent + 1.0f) * std::pow(std::fabs(dot(normal, half)), m_exponent) /
               (8.0f * M_PI * std::fabs(dot(outgoing, half)) * std::fabs(nDotI));
    }
    
protected:
    float m_exponent;
};


// Perfect specular reflection
class PerfectReflection : public Brdf
{
public:
    PerfectReflection() : Brdf() { }
    
    virtual ~PerfectReflection() { }
    
    
    virtual float evaluateSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // Not possible to randomly sample this direction by chance
        outPdf = 0.0f;
        return 0.0f;
    }
    
    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // Not possible to randomly sample this direction by chance
        outPdf = 0.0f;
        return 0.0f;
    }
    
    
    virtual float sampleSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        if (dot(normal, outgoing) < 0.0f)
            outIncoming = outgoing + 2.0f * normal * dot(normal, outgoing);
        else
            outIncoming = outgoing - 2.0f * normal * dot(normal, outgoing);
        outPdf = std::fabs(dot(-outIncoming, normal));
        return 1.0f;
    }
    
    virtual float samplePSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        if (dot(normal, outgoing) < 0.0f)
            outIncoming = outgoing + 2.0f * normal * dot(normal, outgoing);
        else
            outIncoming = outgoing - 2.0f * normal * dot(normal, outgoing);
        outPdf = 1.0f;
        return 1.0f;
    }
    
    
    virtual float pdfSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        return std::fabs(dot(-incoming, normal));
    }
    
    virtual float pdfPSA(const Vector& incoming, const Vector& outgoing, const Vector& normal) const
    {
        return 1.0f;
    }
    
    virtual bool isDiracDistribution() const { return true; }
};

// Perfect specular refraction
class PerfectRefraction : public Btdf
{
public:
    PerfectRefraction() : Btdf() {
        m_ior = 1.0f;
    }

    PerfectRefraction(float ior) : Btdf() {
        m_ior = ior;
    }

    virtual ~PerfectRefraction() { }

    virtual float evaluateSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // Not possible to randomly sample this distribution by chance
        outPdf = 0.0f;
        return 0.0f;
    }

    virtual float evaluatePSA(const Vector& incoming, const Vector& outgoing, const Vector& normal, float& outPdf) const
    {
        // Not possible to randomly sample this distribution by chance
        outPdf = 0.0f;
        return 0.0f;
    }

    virtual float sampleSA(Vector& outIncoming, const Vector& outgoing, const Vector& normal, float u1, float u2, float& outPdf) const
    {
        Vector incident = -outgoing;
        float eta;
        Vector t_norm;
        if(dot(normal, incident) < 0.0f){
            //outside going in
            eta = 1.0f / m_ior;
            t_norm = normal;
        }
        else{
            //inside going out
            eta = m_ior;
            t_norm = -normal;
        }

        float k = 1.0f - eta * eta * (1.0f - dot(t_norm, incident) * dot(t_norm, incident));
        if(k < 0.0f){
            //internal reflection
            outIncoming = incident - 2.0f * t_norm * dot(t_norm, incident);
        }
        else{
            //refraction
            outIncoming = -((incident * eta) - (t_norm * (eta * dot(t_norm, incident) + std::sqrt(k))));
        }

        outPdf = std::fabs(dot(-outIncoming, normal));
        return 1.0f;
    }

    virtual float samplePSA(Vector &outIncoming, const Vector &outgoing, const Vector &normal, float u1, float u2, float &outPdf) const
    {
        sampleSA(outIncoming, outgoing, normal, u1, u2, outPdf);
        outPdf = 1.0f;
        return 1.0f;
    }

    virtual float pdfSA(const Vector &incoming, const Vector &outgoing, const Vector &normal) const
    {
        return std::fabs(dot(incoming, normal));
    }

    virtual float pdfPSA(const Vector &incoming, const Vector &outgoing, const Vector &normal) const
    {
        return 1.0f;
    }

    virtual bool isDiracDistribution() const { return true; }

protected:
    float m_ior;
};

//
// Material
//

class Material
{
public:
    virtual ~Material() { }
    
    // If the material is emitting, override this
    virtual Color emittance() { return Color(); }
    
    virtual Color evaluate(const Point& position,
                           const Vector& normal,
                           const Vector& outgoingRayDirection,
                           Bsdf*& pBsdfChosen,
                           float& bsdfWeight) = 0;
};


// Lambertian diffuse material
class DiffuseMaterial : public Material
{
public:
    DiffuseMaterial(const Color& color) : m_color(color), m_lambert() { }
    
    virtual ~DiffuseMaterial() { }
    
    virtual Color evaluate(const Point& position,
                           const Vector& normal,
                           const Vector& outgoingRayDirection,
                           Bsdf*& pBsdfChosen,
                           float& bsdfWeight)
    {
        bsdfWeight = 1.0f;
        pBsdfChosen = &m_lambert;
        return m_color;
    }
    
protected:
    Color m_color;
    Lambert m_lambert;
};

// Oren-Nayar diffuse material
class OrenNayarMaterial : public Material
{
public:
    OrenNayarMaterial(const Color& color, float sigma) : m_color(color), m_orenNayar(sigma) { }

    virtual ~OrenNayarMaterial() { }

    virtual Color evaluate(const Point &position,
                           const Vector &normal,
                           const Vector &outgoingRayDirection,
                           Bsdf *&pBsdfChosen, float &bsdfWeight)
    {
        bsdfWeight = 1.0f;
        pBsdfChosen = &m_orenNayar;
        return m_color;
    }

protected:
    Color m_color;
    OrenNayar m_orenNayar;
};

// Ashikhmin-Shirley glossy material
class GlossyMaterial : public Material
{
public:
    GlossyMaterial(const Color& color, float roughness) : m_color(color), m_glossy(roughness) { }
    
    virtual ~GlossyMaterial() { }
    
    virtual Color evaluate(const Point& position,
                           const Vector& normal,
                           const Vector& outgoingRayDirection,
                           Bsdf*& pBsdfChosen,
                           float& bsdfWeight)
    {
        bsdfWeight = 1.0f;
        pBsdfChosen = &m_glossy;
        return m_color;
    }
    
protected:
    Color m_color;
    Glossy m_glossy;
};


// Perfect reflection material
class ReflectionMaterial : public Material
{
public:
    ReflectionMaterial(const Color& color) : m_color(color) { }
    
    virtual ~ReflectionMaterial() { }
    
    virtual Color evaluate(const Point& position,
                           const Vector& normal,
                           const Vector& outgoingRayDirection,
                           Bsdf*& pBsdfChosen,
                           float& bsdfWeight)
    {
        bsdfWeight = 1.0f;
        pBsdfChosen = &m_refl;
        return m_color;
    }
    
protected:
    Color m_color;
    PerfectReflection m_refl;
};

// Perfect refraction material
class RefractionMaterial : public Material
{
public:
    RefractionMaterial(const Color& color, const float ior) : m_color(color), m_refr(ior) { }

    virtual ~RefractionMaterial() { }

    virtual Color evaluate(const Point &position,
                           const Vector &normal,
                           const Vector &outgoingRayDirection,
                           Bsdf *&pBsdfChosen,
                           float &bsdfWeight)
    {
        bsdfWeight = 1.0f;
        pBsdfChosen = &m_refr;
        return m_color;
    }

protected:
    Color m_color;
    PerfectRefraction m_refr;
};

// Emitter (light) material
class Emitter : public Material
{
public:
    Emitter(const Color& color, float power) : m_color(color), m_power(power) { }
    
    virtual ~Emitter() { }
    
    virtual Color emittance() { return m_color * m_power; }
    
    virtual Color evaluate(const Point& position,
                           const Vector& normal,
                           const Vector& incomingRayDirection,
                           Bsdf*& pBsdfChosen,
                           float& bsdfWeight)
    {
        // Let the emittance take care of business
        pBsdfChosen = NULL;
        bsdfWeight = 1.0f;
        return Color();
    }
    
protected:
    Color m_color;
    float m_power;
};


} // namespace Rayito


#endif // __RMATERIAL_H__
