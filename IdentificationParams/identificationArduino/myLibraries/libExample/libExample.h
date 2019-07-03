/*
Projet S1 2019
Exemple de librairie pouvant etre ajoute au projet
@author Jean-Samuel Lauzon
@version 1.0 22/05/2019
*/

#ifndef LibExample_H_
#define LibExample_H_

class MyClass
{
    public:
    MyClass();
    ~MyClass();
    void myPublicFunction();
    
    protected:
    void myProtectedFunction();
    
    private:
    void myPrivateFunction();
    int myInt = 0;
    bool myBool = false;
};
#endif // LibExample_H_