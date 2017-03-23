#include <iostream>
class Parent{
	public:
		Parent();
		int pub_var;
		void set_pri_var(int val);
		int get_pri_var();
	private:
		int pri_var;
	protected:
		int pro_var;
};
Parent::Parent(){
	this->pri_var = 0;
}
void Parent::set_pri_var(int val){
	this->pri_var = val;
}
int Parent::get_pri_var(){
	return this->pri_var;
}
class Child : public Parent {
	public:
		Child();
		void set_pro_var(int val);
		int get_pro_var();
};

Child::Child(){
	this->pub_var = 0;
	this->pro_var = 0;
}
void Child::set_pro_var(int val){
	this->pro_var = val;
}
int Child::get_pro_var(){
	return this->pro_var;
}

int main(){
	Child *child = new Child();
	std::cout << "Inherited public: " << child->pub_var << std::endl;
	std::cout << "Inherited protected: " << child->get_pro_var() << std::endl;
	std::cout << "Changing protected to 5\n";
	child->set_pro_var(5);
	std::cout << "New inherited protected: " << child->get_pro_var() << std::endl;
	std::cout << "Private (non-accesible from child)\n";
	std::cout << "Making parent instance\n";
	Parent *parent = new Parent();
	std::cout << "Parent private: " << parent->get_pri_var() << std::endl;
	std::cout << "Setting private to 10\n";
	parent->set_pri_var(10);
	std::cout << "Parent private: " << parent->get_pri_var() << std::endl;
	delete child;
	delete parent;
	return 0;
}
