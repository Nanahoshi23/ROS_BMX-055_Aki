#ifndef __QUATERNION_H__
#define __QUATERNION_H__

#include <Vector3D.h>

class Quaternion
{
    private:
        float m_qw ;
        float m_qx ;
        float m_qy ;
        float m_qz ;

    public:
        Quaternion() ;
        Quaternion(float qw, float qx, float qy, float qz) ;
        Quaternion(Vector3D unit_vector, float radian_angle) ;
        Quaternion(Vector3D base_vector, Vector3D target_vector) ;
        Quaternion(Vector3D base_vector, Vector3D target_vector, float alpha) ;
        
        Quaternion operator = (const Quaternion& other) ;
        Quaternion operator * (const Quaternion& other) const;
        

        Quaternion normalize();
        Vector3D rotateVector3D(Vector3D base_vector);

        float getQW() const;
        float getQX() const;
        float getQY() const;
        float getQZ() const;

        void setQuaternion(float qw, float qx, float qy, float qz) ;
        void setQW(float qw) ;
        void setQX(float qx) ;
        void setQY(float qy) ;
        void setQZ(float qz) ;
} ;


#endif
