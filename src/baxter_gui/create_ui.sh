{ # this is my bash try block
	
	pyside-uic baxtergui.ui > baxtergui.py
	echo "Regenerating baxtergui.py"
} || { # this is catch block
	echo "Install missing pyside tools by:"
	echo "sudo apt-get install pyside-tools"
}
