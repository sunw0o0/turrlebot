# turtle_gui: 파라미터 값을 바꾼 뒤 W/A/S/D 가 주행 대신 입력칸에 글자로 들어가던 것
#   (1) Enter / Esc 로 입력을 끝내면 입력칸에서 빠져나옴 -> 바로 키보드 주행
#   (2) 입력칸 밖 (화면, 빈 곳 등) 을 클릭해도 빠져나옴
#   (3) 숫자 칸 (spin box) 에 커서가 있어도 W/A/S/D 는 주행으로 (숫자 칸에는 어차피 글자가 안 들어감)
#   노트북의 turtle_gui 폴더 (colcon_ws/src/turtle_gui) 에서 실행
import sys
n = 'src/main_window.cpp'
ns = open(n).read()
if 'is_text_input' in ns:
    print('이미 들어가 있어요'); sys.exit()
A = '''bool MainWindow::eventFilter(QObject * obj, QEvent * event)
{
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    const QWidget * focus = QApplication::focusWidget();
    if (qobject_cast<const QLineEdit *>(focus) || qobject_cast<const QAbstractSpinBox *>(focus)) {
      return QMainWindow::eventFilter(obj, event);
    }
    auto * ke = static_cast<QKeyEvent *>(event);
    const int key = ke->key();
'''
B = '''// 글자 입력칸 (파라미터 값 / 검색칸). spin box 안쪽의 글자칸도 spin box 로 친다
static bool is_text_input(const QWidget * w)
{
  return w && (qobject_cast<const QLineEdit *>(w) || qobject_cast<const QAbstractSpinBox *>(w) ||
         (w->parentWidget() && qobject_cast<const QAbstractSpinBox *>(w->parentWidget())));
}

static bool is_number_input(const QWidget * w)
{
  return w && (qobject_cast<const QAbstractSpinBox *>(w) ||
         (w->parentWidget() && qobject_cast<const QAbstractSpinBox *>(w->parentWidget())));
}

bool MainWindow::eventFilter(QObject * obj, QEvent * event)
{
  // 입력칸 밖을 누르면 입력칸에서 빠져나온다 (값은 그때 보내짐). 그래야 W/A/S/D 가 주행으로 간다
  if (event->type() == QEvent::MouseButtonPress) {
    QWidget * focus = QApplication::focusWidget();
    auto * w = qobject_cast<QWidget *>(obj);
    if (is_text_input(focus) && w && w != focus && !focus->isAncestorOf(w) && !w->isAncestorOf(focus) &&
      !is_text_input(w))
    {
      focus->clearFocus();
    }
  }
  if (event->type() == QEvent::KeyPress || event->type() == QEvent::KeyRelease) {
    QWidget * focus = QApplication::focusWidget();
    auto * ke = static_cast<QKeyEvent *>(event);
    const int key = ke->key();
    const bool wasd = key == Qt::Key_W || key == Qt::Key_A || key == Qt::Key_S || key == Qt::Key_D;
    // 숫자 칸에는 W/A/S/D 가 어차피 안 들어가니 주행으로. 글자칸이면 그 칸에 글자를 넘긴다
    if (is_text_input(focus) && !(wasd && is_number_input(focus))) {
      if (event->type() == QEvent::KeyPress &&
        (key == Qt::Key_Return || key == Qt::Key_Enter || key == Qt::Key_Escape))
      {
        // Enter / Esc 로 입력을 끝내면 (값을 보낸 다음) 입력칸에서 빠져나와 바로 키보드 주행이 되게
        QPointer<QWidget> f(focus);
        QTimer::singleShot(0, this, [f] {
            if (f) {
              f->clearFocus();
            }
          });
      }
      return QMainWindow::eventFilter(obj, event);
    }
'''
if ns.count(A) != 1:
    print('파일이 예상과 달라요. 바꾸지 않았어요'); sys.exit()
ns = ns.replace(A, B)
if '#include <QPointer>' not in ns:
    ns = ns.replace('#include <QPlainTextEdit>\n', '#include <QPlainTextEdit>\n#include <QPointer>\n', 1)
open(n, 'w').write(ns)
print('GUI 키보드 주행 고침 OK')
