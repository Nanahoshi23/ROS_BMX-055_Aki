#include <Quaternion.h>
#include <math.h>


Quaternion::Quaternion()
{
    *this = Quaternion(Vector3D(1,0,0),0) ;
}


Quaternion::Quaternion(float qw, float qx, float qy, float qz)
{
    this->setQuaternion(qw, qx, qy, qz) ;
}


Quaternion::Quaternion(Vector3D unit_vector, float radian_angle)
{
    this->setQW(cos(radian_angle/2.0)) ;
    this->setQX(unit_vector.getX() * sin(radian_angle/2.0)) ;
    this->setQY(unit_vector.getY() * sin(radian_angle/2.0)) ;
    this->setQZ(unit_vector.getZ() * sin(radian_angle/2.0)) ;
}


Quaternion::Quaternion(Vector3D base_vector, Vector3D target_vector)
{
    *this = Quaternion((base_vector.cross(target_vector)).normalize(), -acos((base_vector.dot(target_vector))/(base_vector.getNorm()*target_vector.getNorm()))) ;
}


Quaternion::Quaternion(Vector3D base_vector, Vector3D target_vector, float alpha)
{
    *this = Quaternion((base_vector.cross(target_vector)).normalize(), -acos((base_vector.dot(target_vector))/(base_vector.getNorm()*target_vector.getNorm()))*alpha) ;
}

Quaternion Quaternion::operator = (const Quaternion& other)
{
    this->setQuaternion(other.getQW(), other.getQX(), other.getQY(), other.getQZ()) ;
    return *this ;
}


Quaternion Quaternion::operator * (const Quaternion& other) const
{
    return Quaternion(  - this->getQX()*other.getQX() - this->getQY()*other.getQY() - this->getQZ()*other.getQZ() + this->getQW()*other.getQW(),
                        + this->getQW()*other.getQX() - this->getQZ()*other.getQY() + this->getQY()*other.getQZ() + this->getQX()*other.getQW(),
                        + this->getQZ()*other.getQX() + this->getQW()*other.getQY() - this->getQX()*other.getQZ() + this->getQY()*other.getQW(),
                        - this->getQY()*other.getQX() + this->getQX()*other.getQY() + this->getQW()*other.getQZ() + this->getQZ()*other.getQW()) ;
}


Quaternion Quaternion::normalize()
{
    float scalar = sqrt(this->getQW()*this->getQW() + this->getQX()*this->getQX() + this->getQY()*this->getQY() + this->getQZ()*this->getQZ()) ;
    return Quaternion(this->getQW()/scalar, this->getQX()/scalar, this->getQY()/scalar, this->getQZ()/scalar) ;
}


Vector3D Quaternion::rotateVector3D(Vector3D base_vector)
{
    float qw = this->getQW() ;
    float qx = this->getQX() ;
    float qy = this->getQY() ;
    float qz = this->getQZ() ;

    // DCM
    // | dcm00 dcm01 dcm02 |
    // | dcm10 dcm11 dcm12 |
    // | dcm20 dcm21 dcm22 |

    float dcm00 = 1 - 2*(qy*qy + qz*qz) ;
    float dcm01 = 2*(qx*qy + qw*qz) ;
    float dcm02 = 2*(qx*qz - qw*qy) ;
    float dcm10 = 2*(qx*qy - qw*qz) ;
    float dcm11 = 1 - 2*(qx*qx + qz*qz) ;
    float dcm12 = 2*(qy*qz + qw*qx) ;
    float dcm20 = 2*(qx*qz + qw*qy) ;
    float dcm21 = 2*(qy*qz - qw*qx) ;
    float dcm22 = 1 - 2*(qx*qx + qy*qy) ;

    float rotated_vector_x = dcm00*base_vector.getX() + dcm01*base_vector.getY() + dcm02*base_vector.getZ() ;
    float rotated_vector_y = dcm10*base_vector.getX() + dcm11*base_vector.getY() + dcm12*base_vector.getZ() ;
    float rotated_vector_z = dcm20*base_vector.getX() + dcm21*base_vector.getY() + dcm22*base_vector.getZ() ;

    Vector3D rotated_vector(rotated_vector_x, rotated_vector_y, rotated_vector_z) ;
    return rotated_vector ;
}


float Quaternion::getQW() const
{
    return this->m_qw ;
}


float Quaternion::getQX() const
{
    return this->m_qx ;
}


float Quaternion::getQY() const
{
    return this->m_qy ;
}


float Quaternion::getQZ() const
{
    return this->m_qz ;
}


void Quaternion::setQuaternion(float qw, float qx, float qy, float qz)
{
    this->m_qw = qw ;
    this->m_qx = qx ;
    this->m_qy = qy ;
    this->m_qz = qz ;
}


void Quaternion::setQW(float qw)
{
    this->m_qw = qw ;
}


void Quaternion::setQX(float qx)
{
    this->m_qx = qx ;
}


void Quaternion::setQY(float qy)
{
    this->m_qy = qy ;
}


void Quaternion::setQZ(float qz)
{
    this->m_qz = qz ;
}



