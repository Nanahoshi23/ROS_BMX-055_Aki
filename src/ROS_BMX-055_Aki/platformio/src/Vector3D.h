#ifndef __VECTOR_3D__
#define __VECTOR_3D__


class Vector3D
{
    private:
        float m_x ;
        float m_y ;
        float m_z ;

    public:
        Vector3D() ;
        Vector3D(float x, float y, float z) ;

        Vector3D operator = (const Vector3D& other) ;
        Vector3D operator += (const Vector3D& other) ;
        Vector3D operator -= (const Vector3D& other) ;
        Vector3D operator *= (const float& scalar) ;
        Vector3D operator /= (const float& scalar) ;
        Vector3D operator + (const Vector3D& other) const;
        Vector3D operator - (const Vector3D& other) const;
        Vector3D operator * (const float& scalar) const;
        Vector3D operator / (const float& scalar) const;

        bool operator == (const Vector3D& other) const;
        bool operator != (const Vector3D& other) const;
        bool isZero() const;

        Vector3D normalize() const;
        float dot(Vector3D other) const;
        Vector3D cross(Vector3D other) const;

        float getNorm() const;
        float getX() const;
        float getY() const;
        float getZ() const;

        void setVector3D(float x, float y, float z) ;
        void setX(float x) ;
        void setY(float y) ;
        void setZ(float z) ;

} ;

#endif
