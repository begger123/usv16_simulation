#pragma once

#include <iostream>
#include <sstream>

using namespace std;

class Tokenizer 
{
	protected:
		size_t m_offset;
		const std::string m_string;
		std::string m_token;
		std::string m_delimiters;

	public:
		static const std::string DELIMITERS;
		
		Tokenizer(const std::string& str);
		Tokenizer(const std::string& str, const std::string& delimiters);
		
		bool NextToken();
		bool NextToken(const std::string& delimiters);
		const std::string GetToken() const;
		void Reset();
};