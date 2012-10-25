//
//  ViewModel.cpp
//  BachelorThesis
//
//  Created by Eric Wolter on 9/22/12.
//  Copyright (c) 2012 Eric Wolter. All rights reserved.
//

#include "ViewModel.h"

ViewModel::ViewModel() {}
ViewModel::~ViewModel() {}

void ViewModel::init(Vector4f color)
{   
    glGenBuffers(1, &vertexBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(vertices[0]), &vertices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glGenBuffers(1, &indexBufferObject);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(indices[0]), &indices[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    
    float colorBuffer[vertices.size() * 4];
    
    for (int i = 0; i < vertices.size() * 4; i++)
    {
        colorBuffer[i] = color[i % 4];
    }
    
    glGenBuffers(1, &colorBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
    glBufferData(GL_ARRAY_BUFFER, sizeof(colorBuffer), colorBuffer, GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glGenBuffers(1, &normalBufferObject);
    glBindBuffer(GL_ARRAY_BUFFER, normalBufferObject);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(normals[0]), &normals[0], GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    
    glGenVertexArrays(1, &vertexArrayObject);
    glBindVertexArray(vertexArrayObject);
    
    glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, 0);
    
    glBindBuffer(GL_ARRAY_BUFFER, colorBufferObject);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 0, 0);
    
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indexBufferObject);
    
    glBindBuffer(GL_ARRAY_BUFFER, normalBufferObject);
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, 0 , 0);
    
    glBindVertexArray(0);
    
    centerOfMass = Vector3f::Zero();
    vector<Vector3f>::const_iterator vertexIter;
    for(int i = 0; i < vertices.size(); i+=3)
    {
        Vector3f vertex;
        vertex << vertices[i+0], vertices[i+1], vertices[i+2];
        
        centerOfMass += vertex;
    }
    
    centerOfMass /= (vertices.size() / 3);
}

void ViewModel::render(Affine3f cameraTransform, Vector3f position, Quaternionf orientation, GLuint modelToCameraMatrixUnif)
{
    cameraTransform.translate(position);
    cameraTransform.rotate(orientation.toRotationMatrix());
    cameraTransform.translate(-1 * centerOfMass);
    
    glUniformMatrix4fv(modelToCameraMatrixUnif, 1, GL_FALSE, cameraTransform.data());
    glBindVertexArray(vertexArrayObject);
    glDrawElements(GL_TRIANGLES, (int)indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void ViewModel::render(Affine3f cameraTransform, Vector3f position, Quaternionf orientation, Vector3f scale, GLuint modelToCameraMatrixUnif)
{
    cameraTransform.translate(position);
    cameraTransform.rotate(orientation.toRotationMatrix());
    cameraTransform.translate(-1 * centerOfMass);
    cameraTransform.scale(scale);
    
    glUniformMatrix4fv(modelToCameraMatrixUnif, 1, GL_FALSE, cameraTransform.data());
    glBindVertexArray(vertexArrayObject);
    glDrawElements(GL_TRIANGLES, (int)indices.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}