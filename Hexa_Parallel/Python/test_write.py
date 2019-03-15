import xlwt
a = [1,2,3,4]
b = [3,4,5,6]
c = []
c.append(a)
c.append(b)
book = xlwt.Workbook(encoding="utf-8")
sheet1 = book.add_sheet("Sheet 1")

for i in range(0,len(c)):
    for j in range(0,len(c[0])):
        sheet1.write(i,j,c[i][j])


book.save("trial.xls")
