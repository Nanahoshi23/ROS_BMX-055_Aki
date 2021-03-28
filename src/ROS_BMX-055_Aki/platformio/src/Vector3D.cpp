#include <Vector3D.h>
#include <math.h>

Vector3D::Vector3D()
{
    this->setX(0) ;
    this->setY(0) ;
    this->setZ(0) ;
}


Vector3D::Vector3D(float x, float y, float z)
{
    this->setX(x) ;
    this->setY(y) ;
    this->setZ(z) ;
}


Vector3D Vector3D::operator = (const Vector3D& other)
{
    this->setX(other.getX()) ;
    this->setY(other.getY()) ;
    this->setZ(other.getZ()) ;
    return *this ;
}

Vector3D Vector3D::operator += (const Vector3D& other)
{
    return *this = *this + other ;
}


Vector3D Vector3D::operator -= (const Vector3D& other)
{
    return *this = *this - other ;
}


Vector3D Vector3D::operator *= (const float& scalar)
{
    return *this = *this * scalar ;
}


Vector3D Vector3D::operator /= (const float& scalar)
{
    return *this = *this / scalar ;
}


Vector3D Vector3D::operator + (const Vector3D& other) const
{
    return Vector3D(this->getX() + other.getX(), this->getY() + other.getY(), this->getZ() + other.getZ()) ;
}


Vector3D Vector3D::operator - (const Vector3D& other) const
{
    return Vector3D(this->getX() - other.getX(), this->getY() - other.getY(), this->getZ() - other.getZ()) ;
}


Vector3D Vector3D::operator * (const float& scalar) const
{
    return Vector3D(this->getX() * scalar, this->getY() * scalar, this->getZ() * scalar) ;
}


Vector3D Vector3D::operator / (const float& scalar) const
{
    return Vector3D(this->getX() / scalar, this->getY() / scalar, this->getZ() / scalar) ;
}


bool Vector3D::operator == (const Vector3D& other) const
{
    return false ;
}


bool Vector3D::operator != (const Vector3D& other) const
{
    return false ;
}


bool Vector3D::isZero() const
{
    return false ;
}


Vector3D Vector3D::normalize() const
{
    return *this / this->getNorm() ;
}


float Vector3D::dot(Vector3D other) const
{
    return this->getX() * other.getX() + this->getY() * other.getY() + this->getZ() * other.getZ() ;
}


Vector3D Vector3D::cross(Vector3D other) const
{
    return Vector3D(this->getY() * other.getZ() - this->getZ() * other.getY(),
                    this->getZ() * other.getX() - this->getX() * other.getZ(),
                    this->getX() * other.getY() - this->getY() * other.getX()) ;
}


float Vector3D::getNorm() const
{
    return sqrt(this->getX() * this->getX() + this->getY() * this->getY() + this->getZ() * this->getZ()) ;
}


float Vector3D::getX() const
{
    return this->m_x ;
}


float Vector3D::getY() const
{
    return this->m_y ;
}


float Vector3D::getZ() const
{
    return this->m_z ;
}


void Vector3D::setVector3D(float x, float y, float z)
{
    this->setX(x) ;
    this->setY(y) ;
    this->setZ(z) ;
}


void Vector3D::setX(float x)
{
    this->m_x = x ;
}


void Vector3D::setY(float y)
{
    this->m_y = y ;
}


void Vector3D::setZ(float z)
{
    this->m_z = z ;
}
