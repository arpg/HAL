#ifndef SHAPE_H
#define SHAPE_H


class Shape
{
    public:
        virtual ~Shape(){}
};

class BoxShape : public Shape
{

    public:
        Eigen::Vector3d m_dDims;
        BoxShape( double dWidth, double dHeight, double dLength )
        {
            m_dDims[0] = dWidth;
            m_dDims[1] = dHeight;
            m_dDims[2] = dLength;
        }

        BoxShape(Eigen::Vector3d bounds)
        {
            m_dDims = bounds;
        }
};

class CylinderShape : public Shape
{
    public:
        double m_dRadius;
        double m_dThickness;
        CylinderShape(double t, double r)
        {
            m_dRadius = r;
            m_dThickness = t;
        }
};

#endif // SHAPE_H

