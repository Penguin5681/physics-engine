#pragma once

enum ShapeType {
    SPHERE,
    BOX,
    PLANE,
    CYLINDER
};

class Shape {
public:
    ShapeType type;

    Shape(ShapeType t) : type(t) {}
    virtual ~Shape() {}
};