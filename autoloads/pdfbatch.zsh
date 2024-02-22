#! /usr/bin/zsh
cd ./tmp/doc
for file in *.docx; do
 filename="${file%.*}"
 pandoc "$file" -o "../pdf/${filename}.pdf" -pdf-engine=xelatex
 res=$(ls -la)
 status=$(watch -n 2 ls)
done
if [ -n "$res" ];  then
  rm -r ./*
else
  echo "$status"
fi



# Path: autoloads/pdfmerge.zsh